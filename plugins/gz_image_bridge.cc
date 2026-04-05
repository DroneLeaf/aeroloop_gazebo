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
 *   gz_image_bridge <image_topic> [--msp-port PORT]
 *
 * First frame metadata is printed to stderr:
 *   IMGMETA <width> <height> <pix_fmt>
 *
 * pix_fmt values match ffmpeg names: rgb24, rgba, bgr24, bgra
 *
 * OSD overlay (always enabled):
 *   A background thread connects to Betaflight SITL via MSP over TCP
 *   (default port 5763 = UART3) and queries telemetry at ~10 Hz.  Each
 *   camera frame is composited with an FPV-style OSD before being written
 *   to stdout.  The overlay includes battery voltage, current, RSSI,
 *   flight mode, altitude, throttle, heading, GPS satellites, and flight
 *   timer.  If Betaflight is not running, the OSD elements remain blank.
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
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>

#include <cmath>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#ifdef HAS_SDL2
#include <SDL2/SDL.h>
#endif

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
static bool g_osd_enabled = true;   // OSD always enabled
static int  g_msp_port    = 5763;   // UART3 by default (5760 + uart_number)

// Raw-frame UDP stream — forks ffmpeg to encode H.264 and send mpegts.
static std::string g_stream_dest;      // e.g. "10.0.0.87:5000", empty = disabled
static int         g_stream_fd = -1;   // write-end of pipe to ffmpeg child
static pid_t       g_stream_pid = -1;  // ffmpeg child PID

// Direct display mode — renders frames in an SDL2 window instead of piping
// through ffmpeg.  Eliminates encode/decode overhead for minimum latency.
static bool g_display_mode = false;

// ── Shared memory frame server ──────────────────────────────────────────────
// Exposes the latest raw frame via POSIX shared memory so any local process
// (leaf-tracker, OpenCV, etc.) can mmap it and always read the newest frame
// with zero encoding overhead and zero accumulating latency.
//
// Layout:  ShmHeader (64 bytes) + raw pixel data
// Names:   /gz_cam_<sanitised_topic>       — clean frame (no OSD, always active)
//          /gz_cam_<sanitised_topic>_osd   — post-OSD frame (always active)

struct ShmHeader {
    uint32_t magic;          // 0x475A4652 = "GZFR"
    uint32_t width;
    uint32_t height;
    uint32_t channels;       // 3 = RGB/BGR, 4 = RGBA/BGRA
    uint32_t stride;         // width * channels
    uint32_t frame_size;     // stride * height
    uint64_t sequence;       // monotonic counter — reader polls this
    uint64_t timestamp_ns;   // std::chrono steady_clock
    char     pix_fmt[16];    // e.g. "rgb24", "rgba"
    char     _pad[8];        // align to 64 bytes
};
static_assert(sizeof(ShmHeader) == 64, "ShmHeader must be 64 bytes");

struct ShmSegment {
    std::string name;
    int         fd   = -1;
    uint8_t    *ptr  = nullptr;
    size_t      size = 0;
};

static ShmSegment g_shm_clean;   // pre-OSD clean frame (always active)
static ShmSegment g_shm_osd;     // post-OSD frame (always active)

// Non-blocking stream writer thread — prevents companion stream from blocking
// the main loop.  Uses a single-slot buffer with frame dropping.
static std::mutex              g_stream_mutex;
static std::condition_variable g_stream_cv;
static std::string             g_stream_frame;
static bool                    g_stream_new_frame = false;

// ── Forward ground speed from Gazebo pose ─────────────────────────────────
// Subscribes to the Gazebo dynamic_pose/info topic for the drone model,
// differentiates successive positions (~50 Hz), and projects the horizontal
// velocity onto the body-frame forward (X) axis.

static std::string g_model_name;     // extracted from the image topic path
static std::mutex  g_fwd_mutex;
static double      g_forward_speed_ms = 0.0;

struct PoseTracker {
    double x = 0, y = 0;
    double qw = 1, qx = 0, qy = 0, qz = 0;
    std::chrono::steady_clock::time_point stamp;
    bool valid = false;
};
static PoseTracker g_prev_pose;

static void sigHandler(int) { g_running = false; g_cv.notify_all(); g_stream_cv.notify_all(); }

// ─── Shared memory helpers ───────────────────────────────────────────────────

static std::string shmNameFromTopic(const std::string &topic)
{
    // "/world/demo/model/iris/link/camera_link/sensor/camera/image"
    // → "/gz_cam_iris_camera" (keep model + sensor hints, sanitise)
    std::string name = "/gz_cam";
    // Extract last useful segments
    std::vector<std::string> parts;
    size_t pos = 0;
    while (pos < topic.size()) {
        size_t next = topic.find('/', pos + 1);
        if (next == std::string::npos) next = topic.size();
        std::string seg = topic.substr(pos + 1, next - pos - 1);
        if (!seg.empty()) parts.push_back(seg);
        pos = next;
    }
    // Pick "model" name and "sensor" name if present
    for (size_t i = 0; i < parts.size(); i++) {
        if (parts[i] == "model" && i + 1 < parts.size())
            name += "_" + parts[i + 1];
        if (parts[i] == "sensor" && i + 1 < parts.size())
            name += "_" + parts[i + 1];
    }
    if (name == "/gz_cam") name += "_default";
    // Sanitise: only alnum and underscore, max 255 chars
    for (auto &c : name) {
        if (c != '/' && c != '_' && !isalnum(c)) c = '_';
    }
    if (name.size() > 255) name.resize(255);
    return name;
}

static bool initShmSegment(ShmSegment &seg, uint32_t w, uint32_t h,
                          int channels, const char *pix_fmt)
{
    uint32_t stride     = w * channels;
    uint32_t frame_size = stride * h;
    seg.size = sizeof(ShmHeader) + frame_size;

    shm_unlink(seg.name.c_str());

    seg.fd = shm_open(seg.name.c_str(), O_CREAT | O_RDWR, 0666);
    if (seg.fd < 0) {
        perror("[shm] shm_open");
        return false;
    }
    if (ftruncate(seg.fd, seg.size) < 0) {
        perror("[shm] ftruncate");
        close(seg.fd); seg.fd = -1;
        shm_unlink(seg.name.c_str());
        return false;
    }
    seg.ptr = static_cast<uint8_t*>(
        mmap(nullptr, seg.size, PROT_READ | PROT_WRITE, MAP_SHARED, seg.fd, 0));
    if (seg.ptr == MAP_FAILED) {
        perror("[shm] mmap");
        close(seg.fd); seg.fd = -1;
        shm_unlink(seg.name.c_str());
        seg.ptr = nullptr;
        return false;
    }

    auto *hdr = reinterpret_cast<ShmHeader*>(seg.ptr);
    hdr->magic      = 0x475A4652;  // "GZFR"
    hdr->width      = w;
    hdr->height     = h;
    hdr->channels   = channels;
    hdr->stride     = stride;
    hdr->frame_size = frame_size;
    hdr->sequence   = 0;
    hdr->timestamp_ns = 0;
    std::memset(hdr->pix_fmt, 0, sizeof(hdr->pix_fmt));
    std::strncpy(hdr->pix_fmt, pix_fmt, sizeof(hdr->pix_fmt) - 1);

    fprintf(stderr, "[gz_image_bridge] Shared memory: %s  (%ux%u %s, %zu bytes)\n",
            seg.name.c_str(), w, h, pix_fmt, seg.size);
    return true;
}

static void shmSegmentWrite(ShmSegment &seg, const std::string &frame)
{
    if (!seg.ptr) return;
    auto *hdr = reinterpret_cast<ShmHeader*>(seg.ptr);
    uint8_t *dst = seg.ptr + sizeof(ShmHeader);
    size_t copy_size = std::min(frame.size(), static_cast<size_t>(hdr->frame_size));
    std::memcpy(dst, frame.data(), copy_size);

    __atomic_thread_fence(__ATOMIC_RELEASE);

    hdr->timestamp_ns = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    hdr->sequence++;
}

static void cleanupShmSegment(ShmSegment &seg)
{
    if (seg.ptr && seg.ptr != MAP_FAILED) {
        munmap(seg.ptr, seg.size);
        seg.ptr = nullptr;
    }
    if (seg.fd >= 0) {
        close(seg.fd);
        seg.fd = -1;
    }
    if (!seg.name.empty()) {
        shm_unlink(seg.name.c_str());
    }
}

// ─── Raw LAN stream via ffmpeg child ─────────────────────────────────────────

// Fork ffmpeg to read raw frames on stdin and send H.264 mpegts over UDP.
// Returns the write-end fd, or -1 on failure.
static int spawnStreamFfmpeg(uint32_t w, uint32_t h, const char *pix_fmt,
                             const std::string &dest, pid_t &child_pid)
{
    int pipefd[2];
    if (pipe(pipefd) < 0) {
        perror("[stream] pipe");
        return -1;
    }

    child_pid = fork();
    if (child_pid < 0) {
        perror("[stream] fork");
        close(pipefd[0]);
        close(pipefd[1]);
        return -1;
    }

    if (child_pid == 0) {
        // Child — becomes ffmpeg
        close(pipefd[1]);               // close write end
        dup2(pipefd[0], STDIN_FILENO);  // stdin = read end of pipe
        close(pipefd[0]);

        char size_buf[32];
        snprintf(size_buf, sizeof(size_buf), "%ux%u", w, h);

        std::string udp_url = "udp://" + dest + "?pkt_size=1316";

        execlp("ffmpeg", "ffmpeg",
               "-loglevel", "warning",
               "-f", "rawvideo",
               "-pixel_format", pix_fmt,
               "-video_size", size_buf,
               "-framerate", "30",
               "-i", "-",
               "-an",
               "-c:v", "libx264",
               "-preset", "ultrafast",
               "-tune", "zerolatency",
               "-pix_fmt", "yuv420p",
               "-g", "1",
               "-x264-params", "repeat-headers=1",
               "-b:v", "4M",
               "-f", "mpegts",
               udp_url.c_str(),
               (char *)nullptr);
        // execlp only returns on error
        perror("[stream] execlp ffmpeg");
        _exit(127);
    }

    // Parent
    close(pipefd[0]);  // close read end
    fprintf(stderr, "[gz_image_bridge] Streaming raw %ux%u %s → udp://%s (ffmpeg pid %d)\n",
            w, h, pix_fmt, dest.c_str(), (int)child_pid);
    return pipefd[1];  // write end
}

static void streamWriteFrame(int fd, const std::string &frame)
{
    const char *ptr = frame.data();
    size_t rem = frame.size();
    while (rem > 0) {
        ssize_t n = ::write(fd, ptr, rem);
        if (n > 0) { ptr += n; rem -= n; }
        else if (n < 0) {
            if (errno == EINTR) continue;
            // ffmpeg died or pipe broke — disable streaming
            fprintf(stderr, "[stream] write error: %s — disabling stream\n",
                    strerror(errno));
            close(fd);
            g_stream_fd = -1;
            return;
        }
    }
}

// ─── Non-blocking stream writer thread ───────────────────────────────────────

static void streamWriterThread()
{
    while (g_running)
    {
        std::string frame;
        {
            std::unique_lock<std::mutex> lk(g_stream_mutex);
            g_stream_cv.wait_for(lk, std::chrono::milliseconds(100),
                                 [] { return g_stream_new_frame || !g_running; });
            if (!g_stream_new_frame) continue;
            frame.swap(g_stream_frame);
            g_stream_new_frame = false;
        }
        if (g_stream_fd >= 0)
            streamWriteFrame(g_stream_fd, frame);
    }
}

// ─── SDL2 Direct Display ─────────────────────────────────────────────────────

#ifdef HAS_SDL2

static SDL_Window   *g_sdl_window   = nullptr;
static SDL_Renderer *g_sdl_renderer = nullptr;
static SDL_Texture  *g_sdl_texture  = nullptr;

static Uint32 sdlPixelFormat(const char *pf)
{
    if (strcmp(pf, "rgba")  == 0) return SDL_PIXELFORMAT_RGBA32;
    if (strcmp(pf, "bgr24") == 0) return SDL_PIXELFORMAT_BGR24;
    if (strcmp(pf, "bgra")  == 0) return SDL_PIXELFORMAT_BGRA32;
    return SDL_PIXELFORMAT_RGB24;
}

static bool initDisplay(uint32_t w, uint32_t h, const char *pf)
{
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "[display] SDL_Init failed: %s\n", SDL_GetError());
        return false;
    }

    g_sdl_window = SDL_CreateWindow(
        "FPV", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        static_cast<int>(w), static_cast<int>(h),
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    if (!g_sdl_window) {
        fprintf(stderr, "[display] CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return false;
    }

    // No VSYNC — minimum latency (tearing is acceptable for FPV)
    g_sdl_renderer = SDL_CreateRenderer(g_sdl_window, -1,
                                        SDL_RENDERER_ACCELERATED);
    if (!g_sdl_renderer) {
        fprintf(stderr, "[display] CreateRenderer failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(g_sdl_window);
        SDL_Quit();
        return false;
    }

    g_sdl_texture = SDL_CreateTexture(g_sdl_renderer, sdlPixelFormat(pf),
                                      SDL_TEXTUREACCESS_STREAMING,
                                      static_cast<int>(w), static_cast<int>(h));
    if (!g_sdl_texture) {
        fprintf(stderr, "[display] CreateTexture failed: %s\n", SDL_GetError());
        SDL_DestroyRenderer(g_sdl_renderer);
        SDL_DestroyWindow(g_sdl_window);
        SDL_Quit();
        return false;
    }

    fprintf(stderr, "[display] SDL2 direct display %ux%u %s (zero-latency)\n",
            w, h, pf);
    return true;
}

static void displayFrame(const uint8_t *data, uint32_t w, int ch)
{
    SDL_UpdateTexture(g_sdl_texture, nullptr, data,
                      static_cast<int>(w) * ch);
    SDL_RenderClear(g_sdl_renderer);
    SDL_RenderCopy(g_sdl_renderer, g_sdl_texture, nullptr, nullptr);
    SDL_RenderPresent(g_sdl_renderer);

    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        if (e.type == SDL_QUIT) g_running = false;
    }
}

static void cleanupDisplay()
{
    if (g_sdl_texture)  SDL_DestroyTexture(g_sdl_texture);
    if (g_sdl_renderer) SDL_DestroyRenderer(g_sdl_renderer);
    if (g_sdl_window)   SDL_DestroyWindow(g_sdl_window);
    SDL_Quit();
}

#endif // HAS_SDL2

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
            // MSP_STATUS layout: [0-1] cycleTime, [2-3] i2cErrors,
            // [4-5] sensors, [6-9] flightModeFlags, [10] profileIdx
            t.flight_mode_flags = d[6] | (d[7]<<8) | (d[8]<<16) | (d[9]<<24);
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

    // ── Top-center row 2: CH6 guidance mode (MANUAL / TERMINAL) ──
    const char *guide_mode = "MANUAL";
    uint8_t gm_r = 200, gm_g = 200, gm_b = 200;
    if (t.num_channels >= 6 && t.channels[5] > 2000)
    {
        guide_mode = "TERMINAL"; gm_r = 255; gm_g = 60; gm_b = 60;
    }
    int gmlen = static_cast<int>(strlen(guide_mode));
    int gmx = (fw - gmlen * cw) / 2;
    drawElem(frame, fw, fh, ch_count, gmx, margin + ch + 2, guide_mode, scale,
             gm_r, gm_g, gm_b);

    // ── Top-center row 3: CH9 lock status (shown only when active) ──
    if (t.num_channels >= 9 && t.channels[8] > 2000)
    {
        const char *lock_label = "LOCK";
        int lklen = static_cast<int>(strlen(lock_label));
        int lkx = (fw - lklen * cw) / 2;
        drawElem(frame, fw, fh, ch_count, lkx, margin + (ch + 2) * 2, lock_label, scale,
                 255, 200, 50);
    }

    // ── Center: crosshair ──
    drawOsdStr(frame, fw, fh, ch_count,
               (fw - cw) / 2, (fh - ch) / 2, "+", scale);

    // ── Bottom-left: forward ground speed ──
    {
        double fwd;
        { std::lock_guard<std::mutex> lk(g_fwd_mutex); fwd = g_forward_speed_ms; }
        snprintf(buf, sizeof(buf), "FWD:%+.1fm/s", fwd);
        drawElem(frame, fw, fh, ch_count, margin, fh - margin - ch * 3 - 4, buf, scale);
    }

    // ── Bottom-left row 2: altitude ──
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

// ─── Gazebo pose callback (for forward ground speed) ─────────────────────────

static void onPoseV(const gz::msgs::Pose_V &_msg)
{
    if (g_model_name.empty()) return;

    for (int i = 0; i < _msg.pose_size(); i++)
    {
        const auto &p = _msg.pose(i);
        if (p.name() != g_model_name) continue;

        double x  = p.position().x();
        double y  = p.position().y();
        double qw = p.orientation().w();
        double qx = p.orientation().x();
        double qy = p.orientation().y();
        double qz = p.orientation().z();

        auto now = std::chrono::steady_clock::now();

        if (g_prev_pose.valid)
        {
            double dt = std::chrono::duration<double>(
                now - g_prev_pose.stamp).count();
            if (dt >= 0.02)  // cap velocity updates at ~50 Hz
            {
                double vx = (x - g_prev_pose.x) / dt;
                double vy = (y - g_prev_pose.y) / dt;

                // Body-X (forward) direction in the world frame
                double fx = 1.0 - 2.0 * (qy*qy + qz*qz);
                double fy = 2.0 * (qx*qy + qw*qz);
                double fmag = std::sqrt(fx*fx + fy*fy);
                if (fmag > 1e-6) { fx /= fmag; fy /= fmag; }

                double fwd = vx * fx + vy * fy;

                {
                    std::lock_guard<std::mutex> lk(g_fwd_mutex);
                    g_forward_speed_ms = 0.7 * fwd + 0.3 * g_forward_speed_ms;
                }

                g_prev_pose.x = x;
                g_prev_pose.y = y;
                g_prev_pose.stamp = now;
            }
            // Always keep latest orientation for the projection
            g_prev_pose.qw = qw; g_prev_pose.qx = qx;
            g_prev_pose.qy = qy; g_prev_pose.qz = qz;
        }
        else
        {
            g_prev_pose = {x, y, qw, qx, qy, qz, now, true};
        }
        break;
    }
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
    // Parse arguments: <topic> [--msp-port PORT]
    std::string topic;
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--osd") == 0)
            ;  // accepted for backward compat, OSD is always on
        else if (strcmp(argv[i], "--msp-port") == 0 && i + 1 < argc)
            g_msp_port = atoi(argv[++i]);
        else if (strcmp(argv[i], "--stream") == 0 && i + 1 < argc)
            g_stream_dest = argv[++i];
        else if (strcmp(argv[i], "--display") == 0)
            g_display_mode = true;
        else if (topic.empty())
            topic = argv[i];
    }

    if (topic.empty())
    {
        fprintf(stderr,
            "Usage: %s <image_topic> [--msp-port PORT]\n"
            "  --msp-port N       MSP TCP port (default: 5763 = UART3)\n"
            "  --stream H:P       Stream raw (no OSD) H.264 over UDP to host:port\n"
            "  --display          Render in SDL2 window (zero-latency, no stdout)\n",
            argv[0]);
        return 1;
    }

    std::signal(SIGINT, sigHandler);
    std::signal(SIGTERM, sigHandler);

    // Start MSP telemetry thread (OSD always active)
    fprintf(stderr, "[gz_image_bridge] OSD enabled — MSP port %d\n", g_msp_port);
    std::thread osd_thread(mspThread);

    // Start non-blocking stream writer thread
    std::thread stream_thread;
    if (!g_stream_dest.empty())
        stream_thread = std::thread(streamWriterThread);

#ifdef HAS_SDL2
    if (g_display_mode)
        fprintf(stderr, "[gz_image_bridge] Direct display mode (SDL2)\n");
#else
    if (g_display_mode) {
        fprintf(stderr, "[gz_image_bridge] --display requires SDL2 (compile with -DHAS_SDL2)\n");
        return 1;
    }
#endif

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

    // Subscribe to Gazebo pose topic for forward ground speed computation.
    // Parse image topic (/world/{W}/model/{M}/link/…/sensor/…/image) to
    // extract the world name and model name.
    {
        std::vector<std::string> segs;
        size_t pos = 0;
        while (pos < topic.size()) {
            size_t next = topic.find('/', pos + 1);
            if (next == std::string::npos) next = topic.size();
            std::string s = topic.substr(pos + 1, next - pos - 1);
            if (!s.empty()) segs.push_back(s);
            pos = next;
        }
        std::string world_name;
        for (size_t i = 0; i < segs.size(); i++) {
            if (segs[i] == "world" && i + 1 < segs.size())
                world_name = segs[i + 1];
            if (segs[i] == "model" && i + 1 < segs.size() && g_model_name.empty())
                g_model_name = segs[i + 1];
        }
        if (!world_name.empty() && !g_model_name.empty()) {
            std::string pose_topic = "/world/" + world_name + "/dynamic_pose/info";
            if (node.Subscribe(pose_topic, onPoseV))
                fprintf(stderr, "[gz_image_bridge] Tracking '%s' via %s for FWD speed\n",
                        g_model_name.c_str(), pose_topic.c_str());
        }
    }

    bool meta_printed = false;

    // Determine channel count once metadata is ready.
    int ch_count = 3;

    // Writer loop — pulls the latest frame and writes/displays it.
    // The callback keeps overwriting g_frame_data with the newest image,
    // so we never accumulate a backlog regardless of output speed.
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

            // Spawn ffmpeg stream child now that we know resolution
            if (!g_stream_dest.empty())
                g_stream_fd = spawnStreamFfmpeg(g_width, g_height, g_pix_fmt,
                                               g_stream_dest, g_stream_pid);

#ifdef HAS_SDL2
            // Initialize SDL2 display once we know the frame dimensions
            if (g_display_mode) {
                if (!initDisplay(g_width, g_height, g_pix_fmt)) {
                    fprintf(stderr, "[display] Failed to init — falling back to stdout\n");
                    g_display_mode = false;
                }
            }
#endif

            // Initialize POSIX shared memory (always active)
            {
                std::string base = shmNameFromTopic(topic);
                g_shm_clean.name = base;
                if (!initShmSegment(g_shm_clean, g_width, g_height, ch_count, g_pix_fmt))
                    fprintf(stderr, "[shm] Failed to initialize clean segment\n");
                if (g_osd_enabled) {
                    g_shm_osd.name = base + "_osd";
                    if (!initShmSegment(g_shm_osd, g_width, g_height, ch_count, g_pix_fmt))
                        fprintf(stderr, "[shm] Failed to initialize OSD segment\n");
                }
            }
        }

        // ── Clean SHM (always, pre-OSD frame for CV/tracker consumers) ──
        if (meta_printed)
            shmSegmentWrite(g_shm_clean, frame);

        // ── Raw LAN stream (before OSD so frames are clean) ──
        // Hand off to the dedicated stream writer thread (non-blocking).
        if (g_stream_fd >= 0)
        {
            std::lock_guard<std::mutex> lk(g_stream_mutex);
            g_stream_frame = frame;   // copy pre-OSD frame
            g_stream_new_frame = true;
            g_stream_cv.notify_one();
        }

        // ── OSD composite (in-place, before display/write) ──
        if (g_osd_enabled && meta_printed)
        {
            uint8_t *pixels = reinterpret_cast<uint8_t*>(frame.data());
            renderOsd(pixels, static_cast<int>(g_width),
                      static_cast<int>(g_height), ch_count);
            // OSD SHM — post-OSD frame for display consumers
            shmSegmentWrite(g_shm_osd, frame);
        }

#ifdef HAS_SDL2
        // ── Direct display: upload texture and present (sub-millisecond) ──
        if (g_display_mode && g_sdl_texture)
        {
            displayFrame(reinterpret_cast<const uint8_t*>(frame.data()),
                         g_width, ch_count);
            continue;   // skip stdout — display is the output
        }
#endif

        // ── Stdout pipe to ffmpeg (original path) ──
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

#ifdef HAS_SDL2
    if (g_display_mode) cleanupDisplay();
#endif

    // Stop stream writer thread
    g_stream_cv.notify_all();
    if (stream_thread.joinable()) stream_thread.join();

    // Clean up stream child
    if (g_stream_fd >= 0) { close(g_stream_fd); g_stream_fd = -1; }
    if (g_stream_pid > 0) {
        kill(g_stream_pid, SIGTERM);
        waitpid(g_stream_pid, nullptr, 0);
    }

    // Clean up shared memory
    cleanupShmSegment(g_shm_clean);
    cleanupShmSegment(g_shm_osd);

    if (osd_thread.joinable()) osd_thread.join();
    return 0;
}
