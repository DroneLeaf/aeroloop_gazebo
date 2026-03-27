/*
 * gz_image_bridge - Subscribe to a Gazebo image topic and write raw frames
 *                   to stdout for piping into ffmpeg or other consumers.
 *
 * Uses a single-slot "latest frame" buffer so the gz-transport callback
 * never blocks on stdout.  If the downstream consumer (ffmpeg) can't keep
 * up — e.g. it's waiting for a TCP viewer to connect — old frames are
 * silently dropped instead of accumulating unbounded latency.
 *
 * Usage:
 *   gz_image_bridge <image_topic> [--osd [--msp-port PORT]]
 *
 * First frame metadata is printed to stderr:
 *   IMGMETA <width> <height> <pix_fmt>
 *
 * pix_fmt values match ffmpeg names: rgb24, rgba, bgr24, bgra
 *
 * OSD overlay (optional):
 *   When --osd is given, a background thread connects to Betaflight SITL
 *   via MSP over TCP (default port 5762 = UART2) and queries telemetry at
 *   ~10 Hz.  Each camera frame is composited with an FPV-style OSD before
 *   being written to stdout.  The overlay includes battery voltage, current,
 *   RSSI, flight mode, altitude, throttle, heading, GPS satellites, and
 *   flight timer.
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

#include "osd_font.h"

// ─── Global state ────────────────────────────────────────────────────────────

static std::atomic<bool> g_running{true};

// Single-slot latest-frame buffer — the callback overwrites the previous
// frame instantly, so gz-transport's internal queue never grows.
static std::mutex g_mutex;
static std::condition_variable g_cv;
static std::string g_frame_data;
static bool g_new_frame = false;

// Metadata captured from the first frame.
static std::atomic<bool> g_meta_ready{false};
static uint32_t g_width = 0;
static uint32_t g_height = 0;
static const char *g_pix_fmt = "rgb24";

// OSD configuration
static bool g_osd_enabled = false;
static int  g_msp_port    = 5762;   // UART2 by default (5760 + uart_number)

static void sigHandler(int) { g_running = false; g_cv.notify_all(); }

// ─── MSP Protocol ────────────────────────────────────────────────────────────

enum MspCommand : uint8_t {
    MSP_STATUS    = 101,
    MSP_MOTOR     = 104,
    MSP_RC        = 105,
    MSP_RAW_GPS   = 106,
    MSP_ATTITUDE  = 108,
    MSP_ALTITUDE  = 109,
    MSP_ANALOG    = 110,
};

struct MspResponse {
    uint8_t  cmd;
    std::vector<uint8_t> payload;
    bool valid;
};

// Send an MSP V1 request with no payload: $M< 0x00 cmd checksum
static bool mspSendRequest(int sock, uint8_t cmd)
{
    uint8_t buf[6] = { '$', 'M', '<', 0x00, cmd, static_cast<uint8_t>(0x00 ^ cmd) };
    ssize_t n = ::send(sock, buf, sizeof(buf), MSG_NOSIGNAL);
    return n == static_cast<ssize_t>(sizeof(buf));
}

// Read one MSP V1 response frame from the TCP stream.
static MspResponse mspReadResponse(int sock, int timeout_ms = 500)
{
    MspResponse resp{};
    resp.valid = false;

    enum { IDLE, S1, S2, S3, S4, S5, S6 } state = IDLE;
    uint8_t size = 0, cmd = 0, cksum = 0;
    std::vector<uint8_t> payload;
    int idx = 0;

    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline)
    {
        int remain = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count());
        if (remain <= 0) break;

        struct pollfd pfd = { sock, POLLIN, 0 };
        if (::poll(&pfd, 1, remain) <= 0) break;

        uint8_t b;
        if (::recv(sock, &b, 1, 0) <= 0) break;

        switch (state)
        {
        case IDLE: if (b == '$') state = S1;                        break;
        case S1:   state = (b == 'M') ? S2 : IDLE;                 break;
        case S2:   state = (b == '>' || b == '!') ? S3 : IDLE;     break;
        case S3:   // payload size
            size  = b;
            cksum = b;
            payload.resize(size);
            idx   = 0;
            state = S4;
            break;
        case S4:   // command byte
            cmd    = b;
            cksum ^= b;
            state  = (size > 0) ? S5 : S6;
            break;
        case S5:   // payload bytes
            payload[idx++] = b;
            cksum ^= b;
            if (idx >= size) state = S6;
            break;
        case S6:   // checksum
            if (b == cksum)
            {
                resp.cmd     = cmd;
                resp.payload = std::move(payload);
                resp.valid   = true;
            }
            return resp;
        }
    }
    return resp;
}

// ─── OSD Telemetry ───────────────────────────────────────────────────────────

struct OsdTelemetry
{
    // MSP_ANALOG
    float    vbat       = 0;
    float    amps       = 0;
    uint16_t mah_drawn  = 0;
    uint16_t rssi       = 0;   // 0-1023

    // MSP_ATTITUDE
    float    roll_deg   = 0;
    float    pitch_deg  = 0;
    int16_t  heading    = 0;

    // MSP_ALTITUDE
    float    altitude_m = 0;
    float    vario_ms   = 0;

    // MSP_RC
    uint16_t channels[16] = {};
    uint8_t  num_channels = 0;

    // MSP_MOTOR
    uint16_t motors[8] = {};
    uint8_t  num_motors = 0;

    // MSP_STATUS
    uint32_t flight_mode_flags = 0;
    bool     armed = false;

    // MSP_RAW_GPS
    bool     gps_fix       = false;
    uint8_t  gps_sats      = 0;
    float    gps_speed_ms  = 0;

    // Connection & timer
    bool     connected = false;
    std::chrono::steady_clock::time_point arm_start{};
    int      flight_time_s = 0;
};

static std::mutex      g_telem_mutex;
static OsdTelemetry    g_telem;

// ─── MSP background thread ──────────────────────────────────────────────────

static void parseMspResponse(const MspResponse &r, OsdTelemetry &t)
{
    const uint8_t *d = r.payload.data();
    int len = static_cast<int>(r.payload.size());

    switch (r.cmd)
    {
    case MSP_ANALOG:
        if (len >= 7)
        {
            t.vbat      = d[0] / 10.0f;
            t.mah_drawn = d[1] | (d[2] << 8);
            t.rssi      = d[3] | (d[4] << 8);
            t.amps      = static_cast<int16_t>(d[5] | (d[6] << 8)) / 100.0f;
        }
        break;
    case MSP_ATTITUDE:
        if (len >= 6)
        {
            t.roll_deg  = static_cast<int16_t>(d[0] | (d[1] << 8)) / 10.0f;
            t.pitch_deg = static_cast<int16_t>(d[2] | (d[3] << 8)) / 10.0f;
            t.heading   = static_cast<int16_t>(d[4] | (d[5] << 8));
        }
        break;
    case MSP_ALTITUDE:
        if (len >= 6)
        {
            int32_t alt_cm = d[0] | (d[1]<<8) | (d[2]<<16) | (d[3]<<24);
            t.altitude_m = alt_cm / 100.0f;
            t.vario_ms   = static_cast<int16_t>(d[4] | (d[5]<<8)) / 100.0f;
        }
        break;
    case MSP_RC:
        t.num_channels = static_cast<uint8_t>(std::min(len / 2, 16));
        for (int i = 0; i < t.num_channels; i++)
            t.channels[i] = d[i*2] | (d[i*2+1] << 8);
        break;
    case MSP_MOTOR:
        t.num_motors = static_cast<uint8_t>(std::min(len / 2, 8));
        for (int i = 0; i < t.num_motors; i++)
            t.motors[i] = d[i*2] | (d[i*2+1] << 8);
        break;
    case MSP_STATUS:
        if (len >= 11)
        {
            t.flight_mode_flags = d[4] | (d[5]<<8) | (d[6]<<16) | (d[7]<<24);
            t.armed = (t.flight_mode_flags & 1) != 0;
        }
        break;
    case MSP_RAW_GPS:
        if (len >= 16)
        {
            t.gps_fix      = d[0] != 0;
            t.gps_sats     = d[1];
            t.gps_speed_ms = (d[12] | (d[13] << 8)) / 100.0f;
        }
        break;
    }
}

static void mspThread()
{
    const uint8_t queries[] = {
        MSP_ANALOG, MSP_ATTITUDE, MSP_ALTITUDE, MSP_RC, MSP_MOTOR, MSP_STATUS
    };

    while (g_running)
    {
        // ── Connect ──
        int sock = ::socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) { std::this_thread::sleep_for(std::chrono::seconds(2)); continue; }

        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(static_cast<uint16_t>(g_msp_port));
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

        if (::connect(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            ::close(sock);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }

        int one = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        fprintf(stderr, "[OSD] Connected to Betaflight MSP on port %d\n", g_msp_port);
        {
            std::lock_guard<std::mutex> lk(g_telem_mutex);
            g_telem.connected = true;
        }

        // ── Query loop (10 Hz) ──
        while (g_running)
        {
            OsdTelemetry t;
            t.connected = true;
            bool ok = true;

            for (auto cmd : queries)
            {
                if (!g_running) { ok = false; break; }
                if (!mspSendRequest(sock, cmd)) { ok = false; break; }
                auto r = mspReadResponse(sock, 200);
                if (!r.valid) { ok = false; break; }
                parseMspResponse(r, t);
            }

            if (!ok) break;   // reconnect

            {
                std::lock_guard<std::mutex> lk(g_telem_mutex);
                bool was_armed           = g_telem.armed;
                auto prev_arm_start      = g_telem.arm_start;
                int  prev_flight_time_s  = g_telem.flight_time_s;

                g_telem = t;

                // Flight timer
                if (t.armed && !was_armed)
                    g_telem.arm_start = std::chrono::steady_clock::now();
                else if (t.armed)
                {
                    g_telem.arm_start = prev_arm_start;
                    g_telem.flight_time_s = static_cast<int>(
                        std::chrono::duration_cast<std::chrono::seconds>(
                            std::chrono::steady_clock::now() - prev_arm_start).count());
                }
                else
                {
                    g_telem.arm_start      = prev_arm_start;
                    g_telem.flight_time_s  = prev_flight_time_s;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        ::close(sock);
        {
            std::lock_guard<std::mutex> lk(g_telem_mutex);
            g_telem.connected = false;
        }
        if (g_running)
            std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

// ─── OSD Rendering ───────────────────────────────────────────────────────────

// Draw a single 8×8 glyph scaled by `scale`, in the given color.
static inline void drawChar(uint8_t *frame, int fw, int fh, int ch_count,
                             int x, int y, char c, int scale,
                             uint8_t r, uint8_t g, uint8_t b)
{
    unsigned idx = static_cast<unsigned char>(c) - 0x20u;
    if (idx >= 96u) idx = '?' - 0x20u;
    const uint8_t *glyph = OSD_FONT_8X8[idx];

    for (int row = 0; row < 8; row++)
    {
        uint8_t bits = glyph[row];
        if (!bits) continue;
        for (int col = 0; col < 8; col++)
        {
            if (!(bits & (1 << col))) continue;
            int px0 = x + col * scale;
            int py0 = y + row * scale;
            for (int sy = 0; sy < scale; sy++)
            {
                int py = py0 + sy;
                if (py < 0 || py >= fh) continue;
                for (int sx = 0; sx < scale; sx++)
                {
                    int px = px0 + sx;
                    if (px < 0 || px >= fw) continue;
                    int off = (py * fw + px) * ch_count;
                    frame[off + 0] = r;
                    frame[off + 1] = g;
                    frame[off + 2] = b;
                }
            }
        }
    }
}

// Draw a string with 1-pixel shadow for readability.
static void drawOsdStr(uint8_t *frame, int fw, int fh, int ch_count,
                        int x, int y, const char *s, int scale,
                        uint8_t r = 255, uint8_t g = 255, uint8_t b = 255)
{
    int cw = 8 * scale;
    // Shadow pass (black, offset +1)
    for (int i = 0; s[i]; i++)
        drawChar(frame, fw, fh, ch_count, x + i*cw + 1, y + 1, s[i], scale, 0, 0, 0);
    // Foreground
    for (int i = 0; s[i]; i++)
        drawChar(frame, fw, fh, ch_count, x + i*cw, y, s[i], scale, r, g, b);
}

// Darken a rectangle (50 %) for background contrast.
static void darkenRect(uint8_t *frame, int fw, int fh, int ch_count,
                        int rx, int ry, int rw, int rh)
{
    int x0 = std::max(0, rx),       y0 = std::max(0, ry);
    int x1 = std::min(fw, rx + rw), y1 = std::min(fh, ry + rh);
    for (int py = y0; py < y1; py++)
        for (int px = x0; px < x1; px++)
        {
            int off = (py * fw + px) * ch_count;
            frame[off + 0] >>= 1;
            frame[off + 1] >>= 1;
            frame[off + 2] >>= 1;
        }
}

// Helper: draw a labelled OSD element with dark background.
static void drawElem(uint8_t *frame, int fw, int fh, int ch_count,
                      int x, int y, const char *text, int scale,
                      uint8_t r = 255, uint8_t g = 255, uint8_t b = 255)
{
    int cw = 8 * scale;
    int ch = 8 * scale;
    int tw = static_cast<int>(strlen(text)) * cw;
    darkenRect(frame, fw, fh, ch_count, x - 2, y - 1, tw + 4, ch + 2);
    drawOsdStr(frame, fw, fh, ch_count, x, y, text, scale, r, g, b);
}

// Composite the full OSD onto a raw frame buffer.
static void renderOsd(uint8_t *frame, int fw, int fh, int ch_count)
{
    OsdTelemetry t;
    {
        std::lock_guard<std::mutex> lk(g_telem_mutex);
        t = g_telem;
    }

    int scale;
    if      (fw >= 1280) scale = 3;
    else if (fw >=  640) scale = 2;
    else                 scale = 1;

    int cw     = 8 * scale;        // char width in pixels
    int ch     = 8 * scale;        // char height in pixels
    int margin = 4 * scale;
    char buf[64];

    if (!t.connected)
    {
        const char *msg = "NO TELEMETRY";
        int x = (fw - static_cast<int>(strlen(msg)) * cw) / 2;
        drawElem(frame, fw, fh, ch_count, x, ch, msg, scale, 255, 80, 80);
        return;
    }

    // ── Top-left: roll ──
    snprintf(buf, sizeof(buf), "R:%+.1f", static_cast<double>(t.roll_deg));
    drawElem(frame, fw, fh, ch_count, margin, margin, buf, scale);

    // ── Top-left row 2: pitch ──
    snprintf(buf, sizeof(buf), "P:%+.1f", static_cast<double>(t.pitch_deg));
    drawElem(frame, fw, fh, ch_count, margin, margin + ch + 2, buf, scale);

    // ── Top-right: throttle (from average active motor output) ──
    int thr_pct = 0;
    {
        int sum = 0, active = 0;
        for (int i = 0; i < t.num_motors; i++)
        {
            if (t.motors[i] > 0) { sum += t.motors[i]; active++; }
        }
        if (active > 0)
        {
            thr_pct = (sum / active - 1000) * 100 / 1000;
            thr_pct = std::max(0, std::min(100, thr_pct));
        }
    }
    snprintf(buf, sizeof(buf), "THR:%d%%", thr_pct);
    int tlen = static_cast<int>(strlen(buf));
    drawElem(frame, fw, fh, ch_count,
             fw - margin - tlen * cw, margin, buf, scale);

    // ── Top-right row 2: timer ──
    int secs = t.flight_time_s;
    snprintf(buf, sizeof(buf), "%02d:%02d", secs / 60, secs % 60);
    int tmlen = static_cast<int>(strlen(buf));
    drawElem(frame, fw, fh, ch_count,
             fw - margin - tmlen * cw, margin + ch + 2, buf, scale);

    // ── Top-center: flight mode ──
    const char *mode;
    if      (!t.armed)                    mode = "DISARMED";
    else if (t.flight_mode_flags & 0x02)  mode = "ANGLE";
    else if (t.flight_mode_flags & 0x04)  mode = "HORIZON";
    else                                  mode = "ACRO";
    int mlen = static_cast<int>(strlen(mode));
    int mx = (fw - mlen * cw) / 2;
    if (t.armed)
        drawElem(frame, fw, fh, ch_count, mx, margin, mode, scale, 80, 255, 80);
    else
        drawElem(frame, fw, fh, ch_count, mx, margin, mode, scale, 255, 200, 50);

    // ── Center: crosshair ──
    drawOsdStr(frame, fw, fh, ch_count,
               (fw - cw) / 2, (fh - ch) / 2, "+", scale);

    // ── Bottom-left: altitude ──
    snprintf(buf, sizeof(buf), "ALT:%.1fm", static_cast<double>(t.altitude_m));
    drawElem(frame, fw, fh, ch_count, margin, fh - margin - ch * 2 - 2, buf, scale);

    // ── Bottom-left row 2: vertical speed ──
    snprintf(buf, sizeof(buf), "VS:%+.1fm/s", static_cast<double>(t.vario_ms));
    drawElem(frame, fw, fh, ch_count, margin, fh - margin - ch, buf, scale);

    // ── Bottom-right: heading ──
    {
        static const char *dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
        int di = ((t.heading % 360 + 360 + 22) % 360) / 45;
        snprintf(buf, sizeof(buf), "%s %d", dirs[di], t.heading);
        int hlen = static_cast<int>(strlen(buf));
        drawElem(frame, fw, fh, ch_count,
                 fw - margin - hlen * cw, fh - margin - ch * 2 - 2, buf, scale);
    }

    // ── Bottom-center: heading compass bar ──
    snprintf(buf, sizeof(buf), "HDG:%d", static_cast<int>(t.heading));
    int hclen = static_cast<int>(strlen(buf));
    drawElem(frame, fw, fh, ch_count,
             (fw - hclen * cw) / 2, fh - margin - ch, buf, scale);
}

// ─── Gazebo image callback ───────────────────────────────────────────────────

static const char *pixelFormatStr(gz::msgs::PixelFormatType fmt)
{
    switch (fmt)
    {
    case gz::msgs::RGB_INT8:   return "rgb24";
    case gz::msgs::RGBA_INT8:  return "rgba";
    case gz::msgs::BGR_INT8:   return "bgr24";
    case gz::msgs::BGRA_INT8:  return "bgra";
    case gz::msgs::R_FLOAT32:  return "grayf32le";
    default:                   return "rgb24";
    }
}

// Called by gz-transport on every incoming image message.
// Must return ASAP so the transport queue doesn't grow.
static void onImage(const gz::msgs::Image &_msg)
{
    const std::string &data = _msg.data();
    if (data.empty() || !g_running) return;

    {
        std::lock_guard<std::mutex> lk(g_mutex);
        if (!g_meta_ready.load(std::memory_order_relaxed))
        {
            g_width   = _msg.width();
            g_height  = _msg.height();
            g_pix_fmt = pixelFormatStr(_msg.pixel_format_type());
            g_meta_ready.store(true, std::memory_order_release);
        }
        g_frame_data = data;      // overwrite — drop any unwritten frame
        g_new_frame  = true;
    }
    g_cv.notify_one();
}

// ─── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char **argv)
{
    // Parse arguments: <topic> [--osd [--msp-port PORT]]
    std::string topic;
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--osd") == 0)
            g_osd_enabled = true;
        else if (strcmp(argv[i], "--msp-port") == 0 && i + 1 < argc)
            g_msp_port = atoi(argv[++i]);
        else if (topic.empty())
            topic = argv[i];
    }

    if (topic.empty())
    {
        fprintf(stderr,
            "Usage: %s <image_topic> [--osd [--msp-port PORT]]\n"
            "  --osd          Enable Betaflight OSD overlay\n"
            "  --msp-port N   MSP TCP port (default: 5762 = UART2)\n",
            argv[0]);
        return 1;
    }

    std::signal(SIGINT, sigHandler);
    std::signal(SIGTERM, sigHandler);

    // Start MSP telemetry thread if OSD enabled
    std::thread osd_thread;
    if (g_osd_enabled)
    {
        fprintf(stderr, "[gz_image_bridge] OSD enabled — MSP port %d\n", g_msp_port);
        osd_thread = std::thread(mspThread);
    }

    gz::transport::Node node;

    if (!node.Subscribe(topic, onImage))
    {
        fprintf(stderr, "[gz_image_bridge] Failed to subscribe to %s\n",
                topic.c_str());
        g_running = false;
        if (osd_thread.joinable()) osd_thread.join();
        return 1;
    }

    fprintf(stderr, "[gz_image_bridge] Subscribed to %s — waiting for frames\n",
            topic.c_str());

    bool meta_printed = false;

    // Determine channel count once metadata is ready.
    int ch_count = 3;

    // Writer loop — pulls the latest frame and writes it to stdout.
    // If stdout blocks (ffmpeg waiting for TCP client), the mutex is released
    // so the callback can keep overwriting the buffer with fresh frames.
    while (g_running)
    {
        std::string frame;
        {
            std::unique_lock<std::mutex> lk(g_mutex);
            g_cv.wait_for(lk, std::chrono::milliseconds(100),
                          [] { return g_new_frame || !g_running; });
            if (!g_new_frame) continue;
            frame.swap(g_frame_data);
            g_new_frame = false;
        }

        if (!meta_printed && g_meta_ready.load(std::memory_order_acquire))
        {
            fprintf(stderr, "IMGMETA %u %u %s\n", g_width, g_height, g_pix_fmt);
            fflush(stderr);
            meta_printed = true;

            // Determine channels from pixel format
            if (strcmp(g_pix_fmt, "rgba") == 0 || strcmp(g_pix_fmt, "bgra") == 0)
                ch_count = 4;
            else
                ch_count = 3;
        }

        // ── OSD composite (in-place, before write) ──
        if (g_osd_enabled && meta_printed)
        {
            uint8_t *pixels = reinterpret_cast<uint8_t*>(frame.data());
            renderOsd(pixels, static_cast<int>(g_width),
                      static_cast<int>(g_height), ch_count);
        }

        // Blocking write — while we're stuck here the callback keeps
        // overwriting g_frame_data with the newest image, so we never
        // accumulate a backlog.
        const char *ptr = frame.data();
        size_t remaining = frame.size();
        while (remaining > 0 && g_running)
        {
            ssize_t n = ::write(STDOUT_FILENO, ptr, remaining);
            if (n > 0)
            {
                ptr       += n;
                remaining -= n;
            }
            else if (n < 0)
            {
                if (errno == EINTR) continue;
                // EPIPE or other fatal error
                g_running = false;
                break;
            }
        }
    }

    fprintf(stderr, "[gz_image_bridge] Shutting down\n");
    if (osd_thread.joinable()) osd_thread.join();
    return 0;
}
