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
static uint32_t g_out_width = 640;
static uint32_t g_out_height = 480;

// OSD configuration
static bool g_osd_enabled = true;   // OSD always enabled
static int  g_msp_port    = 5763;   // UART3 by default (5760 + uart_number)

// MAVLink OSD mode (PX4 stack)
static bool g_mavlink_osd  = false;  // --mavlink-osd: use MAVLink UDP instead of MSP TCP
static int  g_mavlink_port = 14560;  // dedicated OSD telemetry port

// Camera pitch (degrees) — used for crosshair Z-axis projection.
static double g_cam_pitch_deg = -80.0;

// ── Target proximity detection ──────────────────────────────────────────────
// When --target-model is set, we track the target's live pose (position +
// orientation) via dynamic_pose/info and check whether the drone is within
// the target's oriented bounding box (OBB).  The drone-target delta is
// rotated into the target's local frame before comparing against the
// half-extents.  The flag latches until a world reset is detected.
//
// For multi-link models (e.g. park_chase orbit rig), the model-level pose
// is at the root (orbit center), not the visual body.  Use --target-link
// to specify the link whose world pose should be tracked.  The link pose
// from dynamic_pose/info is relative to the model, so we compose:
//   link_world_pos  = model_pos + R(model_q) * link_rel_pos
//   link_world_quat = model_q * link_rel_q
//
// Half-extents default to the shahed.glb mesh bounds + ~10% tolerance,
// remapped from mesh-local to model-local frame (link pose = -90° roll).
//   model X = mesh X (wingspan)    model Y = mesh Z (fuselage)    model Z = -mesh Y (thickness)
static std::string g_target_model;          // SDF <name> of the target model
static std::string g_target_link;           // optional: link name within model
static double g_target_bbox_x = 0.792;     // half-extent in model-local X (wingspan) (m)
static double g_target_bbox_y = 1.047;     // half-extent in model-local Y (fuselage) (m)
static double g_target_bbox_z = 0.186;     // half-extent in model-local Z (thickness) (m)
static double g_hit_box_scale = 1.0;       // uniform scale applied to all half-extents
static std::atomic<bool> g_target_reached{false};

struct TargetPose {
    double x = 0, y = 0, z = 0;
    double qw = 1, qx = 0, qy = 0, qz = 0;
    bool valid = false;
};
static std::mutex g_target_mutex;
static TargetPose g_target_pose;

// Raw-frame stream — forks ffmpeg to encode H.264. Two output modes:
//   * UDP mpegts   (--stream host:port)  : g_stream_rtsp = false
//   * RTSP push    (--rtsp rtsp://…)     : g_stream_rtsp = true (push to a
//                                          server e.g. mediamtx; -f rtsp)
// The streamed frame is always the *pre-OSD* (clean) frame.
static std::string g_stream_dest;      // host:port (UDP) or rtsp:// URL, empty = disabled
static bool        g_stream_rtsp = false;       // dest is an RTSP push URL
static int         g_stream_fps = 30;           // encoder input/output framerate
static int         g_stream_width = 0;          // explicit output W (0 = source/out width)
static int         g_stream_height = 0;         // explicit output H (0 = source/out height)
static std::string g_stream_bitrate = "4M";     // libx264 target bitrate (-b:v)
static std::string g_stream_preset  = "ultrafast"; // libx264 -preset
static std::string g_stream_tune    = "zerolatency"; // libx264 -tune
static int         g_stream_fd = -1;   // write-end of pipe to ffmpeg child
static pid_t       g_stream_pid = -1;  // ffmpeg child PID

// White-hot thermal styling: render each frame as luminance grayscale so a
// visible-light camera reads like an analog white-hot thermal feed (--thermal).
static bool        g_thermal = false;

// Direct display mode — renders frames in an SDL2 window instead of piping
// through ffmpeg.  Eliminates encode/decode overhead for minimum latency.
static bool g_display_mode = false;
static bool g_hidden_mode  = false;  // --hidden: create SDL2 window hidden (render to SHM only)

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

// Drone world pose — updated in onPoseV for OSD bearing indicator
static std::mutex  g_drone_mutex;
static double      g_drone_x = 0, g_drone_y = 0, g_drone_z = 0;
static double      g_drone_yaw = 0;   // radians, world frame
static bool        g_drone_valid = false;

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
        const int fps = g_stream_fps > 0 ? g_stream_fps : 30;
        char fps_buf[16];
        snprintf(fps_buf, sizeof(fps_buf), "%d", fps);
        // GOP/keyframe interval. UDP is lossy → all-intra (g=1) so any lost
        // packet can't corrupt later frames. RTSP is over reliable TCP, so a
        // normal ~2s GOP gives MUCH better quality at the same bitrate (g=1 was
        // the main cause of low RTSP quality — every frame an I-frame).
        char gop_buf[16];
        snprintf(gop_buf, sizeof(gop_buf), "%d", g_stream_rtsp ? fps * 2 : 1);

        // Video filter: an explicit stream resolution (scale, for upscaling /
        // stress-testing the sink) when both dims are set, else crop 1px off any
        // odd dim (libx264 + yuv420p need even W/H). Round requested dims down to
        // even.
        std::string vf;
        if (g_stream_width > 0 && g_stream_height > 0) {
            char sc[64];
            snprintf(sc, sizeof(sc), "scale=%d:%d:flags=bicubic",
                     g_stream_width & ~1, g_stream_height & ~1);
            vf = sc;
        } else {
            vf = "crop=trunc(iw/2)*2:trunc(ih/2)*2";
        }

        // Build argv dynamically — fps/bitrate/preset/tune are configurable and
        // the output muxer differs for UDP-mpegts vs RTSP push.
        std::vector<std::string> a = {
            "ffmpeg",
            "-loglevel", "warning",
            "-f", "rawvideo",
            "-pixel_format", pix_fmt,
            "-video_size", size_buf,
            "-framerate", fps_buf,
            "-i", "-",
            "-an",
            // Scale to an explicit resolution, or crop odd dims to even (see vf
            // above) — libx264 + yuv420p require even W/H.
            "-vf", vf,
            "-c:v", "libx264",
            "-preset", g_stream_preset,
            "-pix_fmt", "yuv420p",
            "-g", gop_buf,
            "-x264-params", "repeat-headers=1",
            "-b:v", g_stream_bitrate,
        };
        // -tune is optional: "none"/empty omits it (best raw quality, but adds
        // latency since e.g. zerolatency's no-B-frames constraint is lifted).
        if (!g_stream_tune.empty() && g_stream_tune != "none")
            a.insert(a.end(), {"-tune", g_stream_tune});

        std::string out_url;
        if (g_stream_rtsp) {
            // Push to an RTSP server (e.g. mediamtx). TCP transport is the most
            // robust over loopback / lossy links.
            out_url = dest;
            a.insert(a.end(), {"-rtsp_transport", "tcp", "-f", "rtsp", out_url});
        } else {
            out_url = "udp://" + dest + "?pkt_size=1316";
            a.insert(a.end(), {"-f", "mpegts", out_url});
        }

        std::vector<char *> argv;
        argv.reserve(a.size() + 1);
        for (auto &s : a) argv.push_back(const_cast<char *>(s.c_str()));
        argv.push_back(nullptr);

        execvp("ffmpeg", argv.data());
        // execvp only returns on error
        perror("[stream] execvp ffmpeg");
        _exit(127);
    }

    // Parent
    close(pipefd[0]);  // close read end
    fprintf(stderr, "[gz_image_bridge] Streaming raw %ux%u %s @ %dfps %s → %s%s (ffmpeg pid %d)\n",
            w, h, pix_fmt, g_stream_fps, g_stream_bitrate.c_str(),
            g_stream_rtsp ? "" : "udp://", dest.c_str(), (int)child_pid);
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

// Stretch/copy source frame to a fixed output resolution using nearest-neighbor
// sampling. This intentionally allows non-square-pixel display behavior when
// source aspect ratio differs from output (e.g. 640x286 -> 640x480).
static std::string stretchFrameNearest(const std::string &src,
                                       uint32_t src_w, uint32_t src_h,
                                       uint32_t dst_w, uint32_t dst_h,
                                       int ch_count)
{
    const size_t src_expected = static_cast<size_t>(src_w) * src_h * ch_count;
    if (src.size() < src_expected || src_w == 0 || src_h == 0 || ch_count <= 0)
        return src;
    if (src_w == dst_w && src_h == dst_h)
        return src;

    std::string dst;
    dst.resize(static_cast<size_t>(dst_w) * dst_h * ch_count);

    const uint8_t *s = reinterpret_cast<const uint8_t *>(src.data());
    uint8_t *d = reinterpret_cast<uint8_t *>(&dst[0]);

    for (uint32_t y = 0; y < dst_h; y++) {
        uint32_t sy = static_cast<uint32_t>((static_cast<uint64_t>(y) * src_h) / dst_h);
        if (sy >= src_h) sy = src_h - 1;
        for (uint32_t x = 0; x < dst_w; x++) {
            uint32_t sx = static_cast<uint32_t>((static_cast<uint64_t>(x) * src_w) / dst_w);
            if (sx >= src_w) sx = src_w - 1;
            const size_t so = (static_cast<size_t>(sy) * src_w + sx) * ch_count;
            const size_t doff = (static_cast<size_t>(y) * dst_w + x) * ch_count;
            std::memcpy(d + doff, s + so, ch_count);
        }
    }
    return dst;
}

// White-hot thermal styling: collapse each pixel to its luminance (hot/bright →
// white) so a visible-light render reads like an analog white-hot thermal feed.
// In-place; preserves alpha. ch_count 3 (rgb24/bgr24) or 4 (rgba/bgra).
static void applyThermalWhiteHot(std::string &frame, uint32_t w, uint32_t h,
                                 int ch_count, const char *pix_fmt)
{
    const size_t n = static_cast<size_t>(w) * h;
    if (ch_count < 3 || frame.size() < n * static_cast<size_t>(ch_count)) return;

    // Day → night white-hot tone curve (built once).
    //
    // The visible render is a *daytime* scene: the sky/clouds are bright and the
    // target drone is a dark silhouette against them — the opposite of a thermal
    // feed. A real white-hot LWIR feed of an aerial target at night shows a hot
    // (bright) target against a cold (dark) sky. We approximate that by:
    //   1. inverting luminance      → cold-bright sky becomes dark, the dark
    //                                  warm target becomes bright;
    //   2. a night gamma (>1)        → crushes the (now dark) cold sky toward
    //                                  black so it reads as a cold night sky,
    //                                  while hot targets stay bright;
    //   3. a small black floor + white knee for a crisp white-hot target.
    // (Assumes a sky-facing tracker where the target is darker than the
    //  background, which is the aerial-intercept use case.)
    // FLOOR keeps the cold night sky a dark grey (not pure black) so it reads
    // like a real LWIR background AND stays above downstream dark-frame health
    // checks (e.g. leaf-tracker dark_mean_threshold ~8). NOISE_AMP adds subtle
    // sensor grain so the sky isn't a flat digital gain.
    static const int FLOOR = 22;
    static const int NOISE_AMP = 7;          // ± grey levels
    static uint8_t lut[256];
    static bool lut_ready = false;
    if (!lut_ready) {
        const float gamma = 2.0f;            // higher = darker/colder night sky
        for (int i = 0; i < 256; ++i) {
            float inv = (255.0f - static_cast<float>(i)) / 255.0f;  // invert
            float v = std::pow(inv, gamma);                          // night crush
            int out = FLOOR + static_cast<int>(v * (255 - FLOOR) + 0.5f);
            lut[i] = static_cast<uint8_t>(out < 0 ? 0 : out > 255 ? 255 : out);
        }
        lut_ready = true;
    }

    // Per-frame-varying cheap LCG for grain (Math.random/time not needed).
    static uint32_t seed = 0x9e3779b9u;
    seed += 0x6d2b79f5u;
    uint32_t rng = seed;

    // Pixel byte order: rgba/rgb24 → R first; bgra/bgr24 → B first.
    const bool bgr = (strncmp(pix_fmt, "bgr", 3) == 0);
    uint8_t *p = reinterpret_cast<uint8_t *>(&frame[0]);
    for (size_t i = 0; i < n; ++i) {
        uint8_t *px = p + i * ch_count;
        const uint8_t r = bgr ? px[2] : px[0];
        const uint8_t g = px[1];
        const uint8_t b = bgr ? px[0] : px[2];
        // Rec.601 luma (integer: 77/150/29 ≈ 0.299/0.587/0.114, /256).
        const uint8_t y = static_cast<uint8_t>((77 * r + 150 * g + 29 * b) >> 8);
        int t = lut[y];                      // day→night white-hot mapping
        // cheap xorshift-ish grain, ±NOISE_AMP
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        t += static_cast<int>(rng % (2 * NOISE_AMP + 1)) - NOISE_AMP;
        const uint8_t o = static_cast<uint8_t>(t < 0 ? 0 : t > 255 ? 255 : t);
        px[0] = o; px[1] = o; px[2] = o;
    }
}

// ─── Non-blocking stream writer thread ───────────────────────────────────────

static void streamWriterThread()
{
    // Owns the ffmpeg pusher lifecycle: spawn once geometry is known, and
    // respawn whenever it dies (RTSP server not up yet, server restarted, or a
    // broken pipe). Attempts are throttled so a down server isn't hammered.
    auto last_spawn = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    const auto kRespawnCooldown = std::chrono::seconds(2);

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

        // Reap a dead pusher so it can be respawned.
        if (g_stream_pid > 0) {
            int st = 0;
            if (waitpid(g_stream_pid, &st, WNOHANG) == g_stream_pid) {
                if (g_stream_fd >= 0) { close(g_stream_fd); g_stream_fd = -1; }
                g_stream_pid = -1;
            }
        }

        // (Re)spawn once the old pusher is fully reaped and geometry is known,
        // throttled by cooldown (g_stream_pid<=0 avoids double-spawning while a
        // write-error closed the fd but the child hasn't been reaped yet).
        if (g_stream_fd < 0 && g_stream_pid <= 0 &&
            g_meta_ready.load(std::memory_order_acquire)) {
            auto now = std::chrono::steady_clock::now();
            if (now - last_spawn >= kRespawnCooldown) {
                last_spawn = now;
                g_stream_fd = spawnStreamFfmpeg(g_out_width, g_out_height, g_pix_fmt,
                                                g_stream_dest, g_stream_pid);
            }
        }

        if (g_stream_fd >= 0)
            streamWriteFrame(g_stream_fd, frame);
    }

    // Shutdown: stop the pusher.
    if (g_stream_pid > 0) {
        kill(g_stream_pid, SIGTERM);
        waitpid(g_stream_pid, nullptr, 0);
        g_stream_pid = -1;
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

    Uint32 win_flags = SDL_WINDOW_RESIZABLE;
    if (g_hidden_mode)
        win_flags |= SDL_WINDOW_HIDDEN;
    else
        win_flags |= SDL_WINDOW_SHOWN;

    g_sdl_window = SDL_CreateWindow(
        "FPV", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        static_cast<int>(w), static_cast<int>(h),
        win_flags);
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

    // Direct throttle percentage (used by MAVLink OSD, VFR_HUD)
    int      throttle_pct = 0;

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

// ─── MAVLink UDP OSD thread (PX4 stack) ──────────────────────────────────────
// Listens for MAVLink v1/v2 UDP datagrams from PX4 SITL and populates
// the same OsdTelemetry struct used by the MSP path.
// Messages parsed: HEARTBEAT (0), ATTITUDE (30), VFR_HUD (74).

static void mavlinkThread()
{
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("[MAVLink] socket"); return; }

    int one = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(static_cast<uint16_t>(g_mavlink_port));
    addr.sin_addr.s_addr = INADDR_ANY;

    if (::bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("[MAVLink] bind");
        ::close(sock);
        return;
    }

    fprintf(stderr, "[OSD] MAVLink UDP listener on port %d\n", g_mavlink_port);

    // Track arm-start time locally
    auto arm_start = std::chrono::steady_clock::now();
    int  flight_time_s = 0;
    bool was_armed = false;

    uint8_t buf[300];
    while (g_running)
    {
        struct pollfd pfd = { sock, POLLIN, 0 };
        if (::poll(&pfd, 1, 500) <= 0) continue;

        ssize_t n = ::recvfrom(sock, buf, sizeof(buf), 0, nullptr, nullptr);
        if (n < 8) continue;

        uint32_t msg_id;
        const uint8_t *payload;
        int payload_len;

        if (buf[0] == 0xFE) {
            // MAVLink v1: STX(1) LEN(1) SEQ(1) SYS(1) COMP(1) MSG(1) PAYLOAD(LEN) CRC(2)
            payload_len = buf[1];
            msg_id      = buf[5];
            payload     = buf + 6;
            if (n < 6 + payload_len + 2) continue;
        } else if (buf[0] == 0xFD) {
            // MAVLink v2: STX(1) LEN(1) INC(1) CMP(1) SEQ(1) SYS(1) COMP(1) MSG(3) PAYLOAD(LEN) CRC(2)
            payload_len = buf[1];
            msg_id      = buf[7] | (static_cast<uint32_t>(buf[8]) << 8)
                                 | (static_cast<uint32_t>(buf[9]) << 16);
            payload     = buf + 10;
            if (n < 10 + payload_len + 2) continue;
        } else {
            continue;
        }

        {
            std::lock_guard<std::mutex> lk(g_telem_mutex);
            g_telem.connected = true;

            switch (msg_id)
            {
            case 0:  // HEARTBEAT
                // Wire: custom_mode(4) type(1) autopilot(1) base_mode(1) system_status(1) mavlink_version(1)
                // MAVLink v2 may truncate trailing zeros — zero-fill.
                {
                    uint8_t hb[9] = {};
                    std::memcpy(hb, payload, std::min(payload_len, 9));
                    uint8_t base_mode = hb[6];
                    bool now_armed = (base_mode & 0x80) != 0;  // MAV_MODE_FLAG_SAFETY_ARMED
                    if (now_armed && !was_armed)
                        arm_start = std::chrono::steady_clock::now();
                    if (now_armed)
                        flight_time_s = static_cast<int>(
                            std::chrono::duration_cast<std::chrono::seconds>(
                                std::chrono::steady_clock::now() - arm_start).count());
                    was_armed = now_armed;
                    g_telem.armed = now_armed;
                    g_telem.flight_time_s = flight_time_s;
                }
                break;

            case 30: // ATTITUDE
                // Wire: time_boot_ms(4) roll(4) pitch(4) yaw(4) rollspeed(4) pitchspeed(4) yawspeed(4)
                // MAVLink v2 may truncate trailing zeros — zero-fill.
                {
                    uint8_t att[28] = {};
                    std::memcpy(att, payload, std::min(payload_len, 28));
                    float roll, pitch, yaw;
                    std::memcpy(&roll,  att + 4,  4);
                    std::memcpy(&pitch, att + 8,  4);
                    std::memcpy(&yaw,   att + 12, 4);
                    g_telem.roll_deg  = roll  * 180.0f / static_cast<float>(M_PI);
                    g_telem.pitch_deg = pitch * 180.0f / static_cast<float>(M_PI);
                    // Derive heading from yaw (NED, radians → 0-359°)
                    double yaw_deg = std::fmod(yaw * 180.0 / M_PI + 360.0, 360.0);
                    g_telem.heading = static_cast<int16_t>(yaw_deg);
                }
                break;

            case 74: // VFR_HUD
                // Wire: airspeed(4) groundspeed(4) alt(4) climb(4) heading(2) throttle(2)
                // MAVLink v2 truncates trailing zero bytes — zero-fill missing fields.
                {
                    uint8_t vfr[20] = {};
                    std::memcpy(vfr, payload, std::min(payload_len, 20));
                    float alt, climb;
                    int16_t heading;
                    uint16_t throttle;
                    std::memcpy(&alt,      vfr + 8,  4);
                    std::memcpy(&climb,    vfr + 12, 4);
                    std::memcpy(&heading,  vfr + 16, 2);
                    std::memcpy(&throttle, vfr + 18, 2);
                    g_telem.altitude_m   = alt;
                    g_telem.vario_ms     = climb;
                    g_telem.heading      = heading;
                    g_telem.throttle_pct = static_cast<int>(std::min<uint16_t>(throttle, 100));
                }
                break;

            default:
                break;
            }
        } // release g_telem_mutex before any I/O
    }

    ::close(sock);
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

// ─── Artificial Horizon Indicator ────────────────────────────────────────────

// Draw a thick line between two points with shadow for contrast.
static void drawLine(uint8_t *frame, int fw, int fh, int ch_count,
                     int x0, int y0, int x1, int y1, int thickness,
                     uint8_t r, uint8_t g, uint8_t b)
{
    float dx = static_cast<float>(x1 - x0);
    float dy = static_cast<float>(y1 - y0);
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < 0.5f) {
        if (static_cast<unsigned>(x0) < static_cast<unsigned>(fw) &&
            static_cast<unsigned>(y0) < static_cast<unsigned>(fh)) {
            int off = (y0 * fw + x0) * ch_count;
            frame[off] = r; frame[off+1] = g; frame[off+2] = b;
        }
        return;
    }
    float ux = dx / len, uy = dy / len;
    float nx = -uy,      ny = ux;
    int half = thickness / 2;
    int steps = static_cast<int>(len + 0.5f);
    for (int s = 0; s <= steps; s++) {
        float cx = x0 + ux * s;
        float cy = y0 + uy * s;
        for (int t = -half; t <= half; t++) {
            int px = static_cast<int>(cx + nx * t);
            int py = static_cast<int>(cy + ny * t);
            if (static_cast<unsigned>(px) < static_cast<unsigned>(fw) &&
                static_cast<unsigned>(py) < static_cast<unsigned>(fh)) {
                int off = (py * fw + px) * ch_count;
                frame[off]     = r;
                frame[off + 1] = g;
                frame[off + 2] = b;
            }
        }
    }
}

// Draw a line with a black shadow offset by 1 pixel.
static void drawLineShadowed(uint8_t *frame, int fw, int fh, int ch_count,
                             int x0, int y0, int x1, int y1, int thickness,
                             uint8_t r, uint8_t g, uint8_t b)
{
    drawLine(frame, fw, fh, ch_count, x0+1, y0+1, x1+1, y1+1, thickness, 0, 0, 0);
    drawLine(frame, fw, fh, ch_count, x0, y0, x1, y1, thickness, r, g, b);
}

// Rotate point (px,py) by cos_r/sin_r about origin, then translate to (cx,cy).
static inline void horizProject(float px, float py, float cos_r, float sin_r,
                                float cx, float cy, int &sx, int &sy)
{
    sx = static_cast<int>(cx + px * cos_r - py * sin_r);
    sy = static_cast<int>(cy + px * sin_r + py * cos_r);
}

// Draw the artificial horizon overlay.
// roll_deg/pitch_deg are aircraft attitude from Betaflight MSP_ATTITUDE.
static void drawHorizon(uint8_t *frame, int fw, int fh, int ch_count,
                        float roll_deg, float pitch_deg, int scale)
{
    float cx = fw / 2.0f;
    float cy = fh / 2.0f;

    // Pixels per degree — ±60° spans half the frame height.
    float ppd = fh / 120.0f;

    // Roll: aircraft rolls right → horizon tilts left-up / right-down.
    float roll_rad = roll_deg * static_cast<float>(M_PI) / 180.0f;
    float cos_r = std::cos(roll_rad);
    float sin_r = std::sin(roll_rad);

    // Pitch offset (screen Y-down): nose up → horizon below center.
    float pitch_py = pitch_deg * ppd;

    // Dimensions
    int bar_half  = fw / 4;            // each horizon wing half-width
    int gap       = 12 * scale;        // center gap
    int thickness = std::max(2, scale + 1);

    // ── Horizon bar (cyan, two wings) ──
    int sx0, sy0, sx1, sy1;

    // Left wing
    horizProject(static_cast<float>(-bar_half), pitch_py, cos_r, sin_r, cx, cy, sx0, sy0);
    horizProject(static_cast<float>(-gap),      pitch_py, cos_r, sin_r, cx, cy, sx1, sy1);
    drawLineShadowed(frame, fw, fh, ch_count, sx0, sy0, sx1, sy1, thickness, 0, 230, 230);

    // Right wing
    horizProject(static_cast<float>(gap),       pitch_py, cos_r, sin_r, cx, cy, sx0, sy0);
    horizProject(static_cast<float>(bar_half),  pitch_py, cos_r, sin_r, cx, cy, sx1, sy1);
    drawLineShadowed(frame, fw, fh, ch_count, sx0, sy0, sx1, sy1, thickness, 0, 230, 230);

    // ── Fixed aircraft reference (yellow wings at dead center) ──
    int icx = static_cast<int>(cx);
    int icy = static_cast<int>(cy);
    int wing = 18 * scale;
    drawLineShadowed(frame, fw, fh, ch_count,
                     icx - gap - wing, icy, icx - gap, icy, thickness, 255, 220, 0);
    drawLineShadowed(frame, fw, fh, ch_count,
                     icx + gap, icy, icx + gap + wing, icy, thickness, 255, 220, 0);
    // Center dot
    drawLineShadowed(frame, fw, fh, ch_count,
                     icx - 1, icy, icx + 1, icy, thickness + 1, 255, 220, 0);
}

// Composite the full OSD onto a raw frame buffer.
static void renderOsd(uint8_t *frame, int fw, int fh, int ch_count)
{
    OsdTelemetry t;
    {
        std::lock_guard<std::mutex> lk(g_telem_mutex);
        t = g_telem;
    }

    // ~75% of original scale: 1280+ → 2 (was 3), 640+ → 2 (was 2), else → 1
    int scale;
    if      (fw >= 1280) scale = 2;
    else if (fw >=  640) scale = 2;
    else                 scale = 1;

    int cw     = 8 * scale;        // char width in pixels
    int ch     = 8 * scale;        // char height in pixels
    int margin = 8 * scale;        // ~2× original margin → elements shifted inward
    char buf[64];

    if (!t.connected)
    {
        const char *msg = "NO TELEMETRY";
        int x = (fw - static_cast<int>(strlen(msg)) * cw) / 2;
        drawElem(frame, fw, fh, ch_count, x, ch, msg, scale, 255, 80, 80);
        return;
    }

    // ── Artificial horizon indicator (drawn first so text overlays on top) ──
    drawHorizon(frame, fw, fh, ch_count, t.roll_deg, t.pitch_deg, scale);

    // ── Top-left: roll ──
    snprintf(buf, sizeof(buf), "R:%+.1f", static_cast<double>(t.roll_deg));
    drawElem(frame, fw, fh, ch_count, margin, margin, buf, scale);

    // ── Top-left row 2: pitch ──
    snprintf(buf, sizeof(buf), "P:%+.1f", static_cast<double>(t.pitch_deg));
    drawElem(frame, fw, fh, ch_count, margin, margin + ch + 2, buf, scale);

    // ── Top-left row 3: heading (moved from bottom-center) ──
    snprintf(buf, sizeof(buf), "HDG:%d", static_cast<int>(t.heading));
    drawElem(frame, fw, fh, ch_count, margin, margin + (ch + 2) * 2, buf, scale);

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

    // ── Bottom-center row 1: flight mode (moved from top-center) ──
    const char *mode;
    if      (!t.armed)                    mode = "DISARMED";
    else if (t.flight_mode_flags & 0x02)  mode = "ANGLE";
    else if (t.flight_mode_flags & 0x04)  mode = "HORIZON";
    else                                  mode = "ACRO";
    int mlen = static_cast<int>(strlen(mode));
    int mx = (fw - mlen * cw) / 2;
    int by_mode = fh - margin - ch;
    if (t.armed)
        drawElem(frame, fw, fh, ch_count, mx, by_mode, mode, scale, 80, 255, 80);
    else
        drawElem(frame, fw, fh, ch_count, mx, by_mode, mode, scale, 255, 200, 50);

    // ── Bottom-center row 2: CH6 guidance mode (MANUAL / TERMINAL) ──
    const char *guide_mode = "MANUAL";
    uint8_t gm_r = 200, gm_g = 200, gm_b = 200;
    if (t.num_channels >= 6 && t.channels[5] > 2000)
    {
        guide_mode = "TERMINAL"; gm_r = 255; gm_g = 60; gm_b = 60;
    }
    int gmlen = static_cast<int>(strlen(guide_mode));
    int gmx = (fw - gmlen * cw) / 2;
    int by_guide = fh - margin - ch * 2 - 2;
    drawElem(frame, fw, fh, ch_count, gmx, by_guide, guide_mode, scale,
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

    // ── Center: crosshair at projected body Z-up axis ──
    // Camera pitch (e.g. -80°) → optical axis is (90+pitch)° from +Z.
    // Body Z-up projects above image center by an amount that depends on vFOV.
    {
        double camPitchRad = g_cam_pitch_deg * M_PI / 180.0;
        constexpr double kHfovRad = 2.0;
        double offAngle = M_PI / 2.0 + camPitchRad;
        double halfVfov = std::atan(std::tan(kHfovRad / 2.0) * fh / (double)fw);
        int dy = static_cast<int>(std::tan(offAngle) / std::tan(halfVfov) * (fh / 2.0));
        int crossX = (fw - cw) / 2;
        int crossY = (fh - ch) / 2 - dy;  // minus → up in image
        drawOsdStr(frame, fw, fh, ch_count, crossX, crossY, "+", scale);
    }

    // ── Bottom-left: forward ground speed ──
    {
        double fwd;
        { std::lock_guard<std::mutex> lk(g_fwd_mutex); fwd = g_forward_speed_ms; }
        snprintf(buf, sizeof(buf), "FWD:%+.1fm/s", fwd);
        drawElem(frame, fw, fh, ch_count, margin, fh - margin - ch * 3 - 4, buf, scale);
    }

    // ── Bottom-left row 2: altitude with climb/descent arrow ──
    {
        float vz = t.vario_ms;
        char arrow = (vz > 0.3f) ? '^' : (vz < -0.3f) ? 'v' : '-';
        snprintf(buf, sizeof(buf), "ALT:%.0fm%c", static_cast<double>(t.altitude_m), arrow);
        uint8_t ar = 255, ag = 255, ab = 255;
        if      (vz >  0.3f) { ar =  80; ag = 255; ab =  80; }  // green = climbing
        else if (vz < -0.3f) { ar = 255; ag =  80; ab =  80; }  // red   = descending
        drawElem(frame, fw, fh, ch_count, margin, fh - margin - ch * 2 - 2, buf, scale, ar, ag, ab);
    }

    // ── Bottom-left row 3: vertical speed ──
    snprintf(buf, sizeof(buf), "VS:%+.1fm/s", static_cast<double>(t.vario_ms));
    drawElem(frame, fw, fh, ch_count, margin, fh - margin - ch, buf, scale);

    // ── Right-center: variometer bar (5 segs each side of zero) ──
    {
        constexpr int N_SEGS    = 5;
        constexpr float VZ_SCALE = 1.0f;   // 1 segment per 1 m/s
        float vz    = t.vario_ms;
        int   filled = std::min(N_SEGS, static_cast<int>(std::round(std::abs(vz) / VZ_SCALE)));
        bool  climb  = (vz >  0.15f);
        bool  sink   = (vz < -0.15f);
        int   bx     = fw - margin - cw;   // rightmost column
        // Vertically centred: segment 0 = zero-line (middle of bar)
        // seg index from top: 0 = top climb segment, N_SEGS = zero, 2*N_SEGS = bottom sink
        int bar_top_y = fh / 2 - N_SEGS * (ch + 2);
        // Label above bar
        drawOsdStr(frame, fw, fh, ch_count, bx, bar_top_y - ch - 2, "VZ", scale, 180, 180, 180);
        for (int row = 0; row <= 2 * N_SEGS; row++) {
            int seg_y   = bar_top_y + row * (ch + 2);
            int seg_idx = N_SEGS - row;   // +N_SEGS at top, 0 at centre, -N_SEGS at bottom
            if (seg_idx == 0) {
                // Centre / zero marker — always drawn
                drawOsdStr(frame, fw, fh, ch_count, bx, seg_y, "-", scale, 255, 200, 50);
            } else if (seg_idx > 0) {
                // Climb side (above zero)
                if (climb && seg_idx <= filled)
                    drawOsdStr(frame, fw, fh, ch_count, bx, seg_y, "|", scale, 80, 255, 80);
            } else {
                // Sink side (below zero)
                if (sink && (-seg_idx) <= filled)
                    drawOsdStr(frame, fw, fh, ch_count, bx, seg_y, "|", scale, 255, 80, 80);
            }
        }
    }

    // ── Bottom-right: heading ──
    {
        static const char *dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
        int di = ((t.heading % 360 + 360 + 22) % 360) / 45;
        snprintf(buf, sizeof(buf), "%s %d", dirs[di], t.heading);
        int hlen = static_cast<int>(strlen(buf));
        drawElem(frame, fw, fh, ch_count,
                 fw - margin - hlen * cw, fh - margin - ch * 2 - 2, buf, scale);
    }

    // ── Bottom-center: heading compass bar moved to top-left stack ──

    // ── Top-right row 2: target bearing / distance indicator ──
    if (!g_target_model.empty())
    {
        double dx, dy, dz, dyaw;
        bool dv;
        { std::lock_guard<std::mutex> lk(g_drone_mutex);
          dx = g_drone_x; dy = g_drone_y; dz = g_drone_z;
          dyaw = g_drone_yaw; dv = g_drone_valid; }

        TargetPose tp;
        { std::lock_guard<std::mutex> lk(g_target_mutex); tp = g_target_pose; }

        if (dv && tp.valid) {
            double wx = tp.x - dx;
            double wy = tp.y - dy;
            double wz = tp.z - dz;
            double horiz_dist = std::sqrt(wx*wx + wy*wy);
            double dist_3d    = std::sqrt(wx*wx + wy*wy + wz*wz);

            // Absolute bearing from drone to target (world frame, degrees)
            double abs_bearing_rad = std::atan2(wy, wx);
            int abs_bearing_deg = static_cast<int>(
                std::fmod(90.0 - abs_bearing_rad * 180.0 / M_PI + 360.0, 360.0));

            // Relative bearing (from drone nose)
            double rel_rad = abs_bearing_rad - dyaw;
            // Normalise to [-pi, pi]
            while (rel_rad >  M_PI) rel_rad -= 2.0 * M_PI;
            while (rel_rad < -M_PI) rel_rad += 2.0 * M_PI;

            // 8-direction arrow character based on relative bearing
            // Negate: positive rel_rad = CCW (left in body) should map to '<'
            static const char *arrows[] = {"^","\\",">","/","v","\\","<","/"};
            int ai = (static_cast<int>(std::round(-rel_rad * 4.0 / M_PI)) + 8) % 8;

            // Distance text: use km if > 1000 m
            char dist_buf[16];
            if (dist_3d >= 1000.0)
                snprintf(dist_buf, sizeof(dist_buf), "%.1fK", dist_3d / 1000.0);
            else
                snprintf(dist_buf, sizeof(dist_buf), "%.0fm", dist_3d);

            // Elevation angle
            int elev_deg = static_cast<int>(std::atan2(wz, horiz_dist) * 180.0 / M_PI);

            snprintf(buf, sizeof(buf), "TGT %s%03d %s %+d",
                     arrows[ai], abs_bearing_deg, dist_buf, elev_deg);
            int tlen = static_cast<int>(strlen(buf));
            drawElem(frame, fw, fh, ch_count,
                     fw - margin - tlen * cw, margin + ch + 2, buf, scale,
                     0, 255, 128);  // green

            // ── Pointer line from screen center toward target direction ──
            int pcx = fw / 2;
            int pcy = fh / 2;
            int ptr_len = std::min(fw, fh) / 5;
            // Screen: UP=forward, RIGHT=right; rel_rad: 0=fwd, +CCW=left
            int px = pcx - static_cast<int>(ptr_len * std::sin(rel_rad));
            int py = pcy - static_cast<int>(ptr_len * std::cos(rel_rad));
            drawLineShadowed(frame, fw, fh, ch_count,
                             pcx, pcy, px, py,
                             std::max(2, scale), 0, 255, 128);
        }
    }

    // ── TARGET REACHED indicator (live — clears when drone leaves bbox) ──
    if (g_target_reached.load(std::memory_order_relaxed))
    {
        const char *hit_msg = "TARGET REACHED";
        int hit_len = static_cast<int>(strlen(hit_msg));
        int hit_x = (fw - hit_len * cw) / 2;
        int hit_y = fh / 2 + ch * 3;  // below center
        // Flashing effect: alternate between bright red and yellow
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        bool flash = ((ms / 500) % 2) == 0;
        uint8_t hr = 255, hg = flash ? 50u : 220u, hb = flash ? 50u : 0u;
        drawElem(frame, fw, fh, ch_count, hit_x, hit_y, hit_msg, scale, hr, hg, hb);
    }
}

// ─── PX4 MAVLink OSD Rendering ──────────────────────────────────────────────
// Simplified OSD for PX4 stack: altitude, heading, roll, yaw, throttle %,
// artificial horizon, armed/disarmed, forward speed, vertical speed.
// No battery, RSSI, flight-mode flags, guidance mode, or lock indicators.

static void renderPx4Osd(uint8_t *frame, int fw, int fh, int ch_count)
{
    OsdTelemetry t;
    {
        std::lock_guard<std::mutex> lk(g_telem_mutex);
        t = g_telem;
    }

    int scale;
    if      (fw >= 1280) scale = 2;
    else if (fw >=  640) scale = 2;
    else                 scale = 1;

    int cw     = 8 * scale;
    int ch     = 8 * scale;
    int margin = 8 * scale;
    char buf[64];

    if (!t.connected)
    {
        const char *msg = "NO TELEMETRY";
        int x = (fw - static_cast<int>(strlen(msg)) * cw) / 2;
        drawElem(frame, fw, fh, ch_count, x, ch, msg, scale, 255, 80, 80);
        return;
    }

    // ── Artificial horizon ──
    drawHorizon(frame, fw, fh, ch_count, t.roll_deg, t.pitch_deg, scale);

    // ── Top-left: roll ──
    snprintf(buf, sizeof(buf), "R:%+.1f", static_cast<double>(t.roll_deg));
    drawElem(frame, fw, fh, ch_count, margin, margin, buf, scale);

    // ── Top-left row 2: pitch ──
    snprintf(buf, sizeof(buf), "P:%+.1f", static_cast<double>(t.pitch_deg));
    drawElem(frame, fw, fh, ch_count, margin, margin + ch + 2, buf, scale);

    // ── Top-right: throttle ──
    snprintf(buf, sizeof(buf), "THR:%d%%", t.throttle_pct);
    int tlen = static_cast<int>(strlen(buf));
    drawElem(frame, fw, fh, ch_count,
             fw - margin - tlen * cw, margin, buf, scale);

    // ── Top-right row 2: timer ──
    int secs = t.flight_time_s;
    snprintf(buf, sizeof(buf), "%02d:%02d", secs / 60, secs % 60);
    int tmlen = static_cast<int>(strlen(buf));
    drawElem(frame, fw, fh, ch_count,
             fw - margin - tmlen * cw, margin + ch + 2, buf, scale);

    // ── Top-center: armed / disarmed ──
    const char *arm_str = t.armed ? "ARMED" : "DISARMED";
    int alen = static_cast<int>(strlen(arm_str));
    int ax = (fw - alen * cw) / 2;
    if (t.armed)
        drawElem(frame, fw, fh, ch_count, ax, margin, arm_str, scale, 80, 255, 80);
    else
        drawElem(frame, fw, fh, ch_count, ax, margin, arm_str, scale, 255, 200, 50);

    // ── Center: crosshair at projected body Z-up axis ──
    {
        double camPitchRad = g_cam_pitch_deg * M_PI / 180.0;
        constexpr double kHfovRad = 2.0;
        double offAngle = M_PI / 2.0 + camPitchRad;
        double halfVfov = std::atan(std::tan(kHfovRad / 2.0) * fh / (double)fw);
        int dy = static_cast<int>(std::tan(offAngle) / std::tan(halfVfov) * (fh / 2.0));
        int crossX = (fw - cw) / 2;
        int crossY = (fh - ch) / 2 - dy;
        drawOsdStr(frame, fw, fh, ch_count, crossX, crossY, "+", scale);
    }

    // ── Bottom-left: forward ground speed (from Gazebo pose) ──
    {
        double fwd;
        { std::lock_guard<std::mutex> lk(g_fwd_mutex); fwd = g_forward_speed_ms; }
        snprintf(buf, sizeof(buf), "FWD:%+.1fm/s", fwd);
        drawElem(frame, fw, fh, ch_count, margin, fh - margin - ch * 3 - 4, buf, scale);
    }

    // ── Bottom-left row 2: altitude with climb/descent arrow ──
    {
        float vz = t.vario_ms;
        char arrow = (vz > 0.3f) ? '^' : (vz < -0.3f) ? 'v' : '-';
        snprintf(buf, sizeof(buf), "ALT:%.0fm%c", static_cast<double>(t.altitude_m), arrow);
        uint8_t ar = 255, ag = 255, ab = 255;
        if      (vz >  0.3f) { ar =  80; ag = 255; ab =  80; }  // green = climbing
        else if (vz < -0.3f) { ar = 255; ag =  80; ab =  80; }  // red   = descending
        drawElem(frame, fw, fh, ch_count, margin, fh - margin - ch * 2 - 2, buf, scale, ar, ag, ab);
    }

    // ── Bottom-left row 3: vertical speed ──
    snprintf(buf, sizeof(buf), "VS:%+.1fm/s", static_cast<double>(t.vario_ms));
    drawElem(frame, fw, fh, ch_count, margin, fh - margin - ch, buf, scale);

    // ── Right-center: variometer bar (5 segs each side of zero) ──
    {
        constexpr int N_SEGS    = 5;
        constexpr float VZ_SCALE = 1.0f;   // 1 segment per 1 m/s
        float vz    = t.vario_ms;
        int   filled = std::min(N_SEGS, static_cast<int>(std::round(std::abs(vz) / VZ_SCALE)));
        bool  climb  = (vz >  0.15f);
        bool  sink   = (vz < -0.15f);
        int   bx     = fw - margin - cw;   // rightmost column
        int bar_top_y = fh / 2 - N_SEGS * (ch + 2);
        drawOsdStr(frame, fw, fh, ch_count, bx, bar_top_y - ch - 2, "VZ", scale, 180, 180, 180);
        for (int row = 0; row <= 2 * N_SEGS; row++) {
            int seg_y   = bar_top_y + row * (ch + 2);
            int seg_idx = N_SEGS - row;
            if (seg_idx == 0) {
                drawOsdStr(frame, fw, fh, ch_count, bx, seg_y, "-", scale, 255, 200, 50);
            } else if (seg_idx > 0) {
                if (climb && seg_idx <= filled)
                    drawOsdStr(frame, fw, fh, ch_count, bx, seg_y, "|", scale, 80, 255, 80);
            } else {
                if (sink && (-seg_idx) <= filled)
                    drawOsdStr(frame, fw, fh, ch_count, bx, seg_y, "|", scale, 255, 80, 80);
            }
        }
    }

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

    // ── Target bearing / distance ──
    if (!g_target_model.empty())
    {
        double dx, dy, dz, dyaw;
        bool dv;
        { std::lock_guard<std::mutex> lk(g_drone_mutex);
          dx = g_drone_x; dy = g_drone_y; dz = g_drone_z;
          dyaw = g_drone_yaw; dv = g_drone_valid; }

        TargetPose tp;
        { std::lock_guard<std::mutex> lk(g_target_mutex); tp = g_target_pose; }

        if (dv && tp.valid) {
            double wx = tp.x - dx;
            double wy = tp.y - dy;
            double wz = tp.z - dz;
            double horiz_dist = std::sqrt(wx*wx + wy*wy);
            double dist_3d    = std::sqrt(wx*wx + wy*wy + wz*wz);

            double abs_bearing_rad = std::atan2(wy, wx);
            int abs_bearing_deg = static_cast<int>(
                std::fmod(90.0 - abs_bearing_rad * 180.0 / M_PI + 360.0, 360.0));

            double rel_rad = abs_bearing_rad - dyaw;
            while (rel_rad >  M_PI) rel_rad -= 2.0 * M_PI;
            while (rel_rad < -M_PI) rel_rad += 2.0 * M_PI;

            static const char *arrows[] = {"^","\\",">","/","v","\\","<","/"};
            int ai = (static_cast<int>(std::round(-rel_rad * 4.0 / M_PI)) + 8) % 8;

            char dist_buf[16];
            if (dist_3d >= 1000.0)
                snprintf(dist_buf, sizeof(dist_buf), "%.1fK", dist_3d / 1000.0);
            else
                snprintf(dist_buf, sizeof(dist_buf), "%.0fm", dist_3d);

            int elev_deg = static_cast<int>(std::atan2(wz, horiz_dist) * 180.0 / M_PI);

            snprintf(buf, sizeof(buf), "TGT %s%03d %s %+d",
                     arrows[ai], abs_bearing_deg, dist_buf, elev_deg);
            int tlen2 = static_cast<int>(strlen(buf));
            drawElem(frame, fw, fh, ch_count,
                     fw - margin - tlen2 * cw, margin + (ch + 2) * 2, buf, scale,
                     0, 255, 128);

            int pcx = fw / 2;
            int pcy = fh / 2;
            int ptr_len = std::min(fw, fh) / 5;
            int px = pcx - static_cast<int>(ptr_len * std::sin(rel_rad));
            int py = pcy - static_cast<int>(ptr_len * std::cos(rel_rad));
            drawLineShadowed(frame, fw, fh, ch_count,
                             pcx, pcy, px, py,
                             std::max(2, scale), 0, 255, 128);
        }
    }

    // ── TARGET REACHED indicator ──
    if (g_target_reached.load(std::memory_order_relaxed))
    {
        const char *hit_msg = "TARGET REACHED";
        int hit_len = static_cast<int>(strlen(hit_msg));
        int hit_x = (fw - hit_len * cw) / 2;
        int hit_y = fh / 2 + ch * 3;
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        bool flash = ((ms / 500) % 2) == 0;
        uint8_t hr = 255, hg = flash ? 50u : 220u, hb = flash ? 50u : 0u;
        drawElem(frame, fw, fh, ch_count, hit_x, hit_y, hit_msg, scale, hr, hg, hb);
    }
}

// ─── Gazebo pose callback (for forward ground speed) ─────────────────────────

static void onPoseV(const gz::msgs::Pose_V &_msg)
{
    if (g_model_name.empty()) return;

    // ── Track target model pose ──
    if (!g_target_model.empty())
    {
        double drone_x = 0, drone_y = 0, drone_z = 0;
        bool   have_drone = false;

        // Model-level pose (world frame) — static: the model root of a
        // multi-link target (e.g. orbit center) may only appear once on
        // dynamic_pose/info because it never moves after spawning.
        static double mdl_x = 0, mdl_y = 0, mdl_z = 0;
        static double mdl_qw = 1, mdl_qx = 0, mdl_qy = 0, mdl_qz = 0;
        static bool   have_model = false;

        // Link-relative pose (relative to model, only when --target-link set)
        // — static: model and link may arrive in different Pose_V messages.
        static double lnk_x = 0, lnk_y = 0, lnk_z = 0;
        static double lnk_qw = 1, lnk_qx = 0, lnk_qy = 0, lnk_qz = 0;
        static bool   have_link = false;

        // Build the scoped link name once (globals are set before subscription)
        static const std::string link_scoped =
            g_target_link.empty() ? std::string()
                                  : g_target_model + "::" + g_target_link;

        for (int i = 0; i < _msg.pose_size(); i++)
        {
            const auto &p = _msg.pose(i);
            const std::string &name = p.name();
            if (name == g_model_name) {
                drone_x = p.position().x();
                drone_y = p.position().y();
                drone_z = p.position().z();
                have_drone = true;
            }
            if (name == g_target_model) {
                mdl_x  = p.position().x();
                mdl_y  = p.position().y();
                mdl_z  = p.position().z();
                mdl_qw = p.orientation().w();
                mdl_qx = p.orientation().x();
                mdl_qy = p.orientation().y();
                mdl_qz = p.orientation().z();
                have_model = true;
            }
            // Flexible link matching: exact "model::link", bare "link",
            // or any name ending with "::model::link" (world-scoped)
            if (!link_scoped.empty()) {
                bool lmatch = (name == link_scoped)
                           || (name == g_target_link)
                           || (name.size() > link_scoped.size() + 2 &&
                               name.compare(name.size() - link_scoped.size(),
                                            link_scoped.size(), link_scoped) == 0);
                if (lmatch) {
                    lnk_x  = p.position().x();
                    lnk_y  = p.position().y();
                    lnk_z  = p.position().z();
                    lnk_qw = p.orientation().w();
                    lnk_qx = p.orientation().x();
                    lnk_qy = p.orientation().y();
                    lnk_qz = p.orientation().z();
                    have_link = true;
                }
            }
        }

        // Resolve effective target world pose
        double tgt_x, tgt_y, tgt_z;
        double tgt_qw, tgt_qx, tgt_qy, tgt_qz;
        bool have_target = false;

        if (!link_scoped.empty()) {
            // Multi-link model: compose model + link-relative
            if (have_model && have_link) {
                // pos_world = model_pos + R(model_q) * link_rel_pos
                double rx = lnk_x * (1.0 - 2.0*(mdl_qy*mdl_qy + mdl_qz*mdl_qz))
                          + lnk_y * (2.0*(mdl_qx*mdl_qy - mdl_qw*mdl_qz))
                          + lnk_z * (2.0*(mdl_qx*mdl_qz + mdl_qw*mdl_qy));
                double ry = lnk_x * (2.0*(mdl_qx*mdl_qy + mdl_qw*mdl_qz))
                          + lnk_y * (1.0 - 2.0*(mdl_qx*mdl_qx + mdl_qz*mdl_qz))
                          + lnk_z * (2.0*(mdl_qy*mdl_qz - mdl_qw*mdl_qx));
                double rz = lnk_x * (2.0*(mdl_qx*mdl_qz - mdl_qw*mdl_qy))
                          + lnk_y * (2.0*(mdl_qy*mdl_qz + mdl_qw*mdl_qx))
                          + lnk_z * (1.0 - 2.0*(mdl_qx*mdl_qx + mdl_qy*mdl_qy));
                tgt_x = mdl_x + rx;
                tgt_y = mdl_y + ry;
                tgt_z = mdl_z + rz;
                // quat_world = model_q * link_rel_q  (Hamilton product)
                tgt_qw = mdl_qw*lnk_qw - mdl_qx*lnk_qx - mdl_qy*lnk_qy - mdl_qz*lnk_qz;
                tgt_qx = mdl_qw*lnk_qx + mdl_qx*lnk_qw + mdl_qy*lnk_qz - mdl_qz*lnk_qy;
                tgt_qy = mdl_qw*lnk_qy - mdl_qx*lnk_qz + mdl_qy*lnk_qw + mdl_qz*lnk_qx;
                tgt_qz = mdl_qw*lnk_qz + mdl_qx*lnk_qy - mdl_qy*lnk_qx + mdl_qz*lnk_qw;
                have_target = true;
            }
        } else {
            // Simple model: model pose IS the target
            if (have_model) {
                tgt_x = mdl_x;  tgt_y = mdl_y;  tgt_z = mdl_z;
                tgt_qw = mdl_qw; tgt_qx = mdl_qx; tgt_qy = mdl_qy; tgt_qz = mdl_qz;
                have_target = true;
            }
        }

        if (have_target) {
            std::lock_guard<std::mutex> lk(g_target_mutex);
            g_target_pose = {tgt_x, tgt_y, tgt_z,
                             tgt_qw, tgt_qx, tgt_qy, tgt_qz, true};
        }

        if (have_drone && have_target) {
            // Delta in world frame
            double wx = drone_x - tgt_x;
            double wy = drone_y - tgt_y;
            double wz = drone_z - tgt_z;

            // Detect world reset: if drone teleports > 10 m in one pose update,
            // clear the latch so a new run can trigger it again.
            {
                static double prev_dx = 0, prev_dy = 0, prev_dz = 0;
                static bool prev_valid = false;
                double jx = drone_x - prev_dx;
                double jy = drone_y - prev_dy;
                double jz = drone_z - prev_dz;
                double jump = jx*jx + jy*jy + jz*jz;
                prev_dx = drone_x; prev_dy = drone_y; prev_dz = drone_z;
                if (prev_valid && jump > 100.0) {  // > 10 m jump
                    if (g_target_reached.load(std::memory_order_relaxed)) {
                        fprintf(stderr, "[gz_image_bridge] Reset detected (jump=%.1f m), clearing TARGET REACHED latch\n", std::sqrt(jump));
                        g_target_reached.store(false, std::memory_order_release);
                    }
                }
                prev_valid = true;
            }

            // Rotate delta into target's local frame using inverse(target_quat).
            // For a unit quaternion, inverse = conjugate (w, -x, -y, -z).
            // R^T * v  where R is the rotation matrix from the quaternion:
            double lx = wx * (1.0 - 2.0*(tgt_qy*tgt_qy + tgt_qz*tgt_qz))
                      + wy * (2.0*(tgt_qx*tgt_qy + tgt_qw*tgt_qz))
                      + wz * (2.0*(tgt_qx*tgt_qz - tgt_qw*tgt_qy));
            double ly = wx * (2.0*(tgt_qx*tgt_qy - tgt_qw*tgt_qz))
                      + wy * (1.0 - 2.0*(tgt_qx*tgt_qx + tgt_qz*tgt_qz))
                      + wz * (2.0*(tgt_qy*tgt_qz + tgt_qw*tgt_qx));
            double lz = wx * (2.0*(tgt_qx*tgt_qz + tgt_qw*tgt_qy))
                      + wy * (2.0*(tgt_qy*tgt_qz - tgt_qw*tgt_qx))
                      + wz * (1.0 - 2.0*(tgt_qx*tgt_qx + tgt_qy*tgt_qy));

            bool inside = std::abs(lx) <= g_target_bbox_x * g_hit_box_scale
                       && std::abs(ly) <= g_target_bbox_y * g_hit_box_scale
                       && std::abs(lz) <= g_target_bbox_z * g_hit_box_scale;
            bool was_inside = g_target_reached.load(std::memory_order_relaxed);

            // Throttled debug: print proximity info once/sec when within 5 m
            double dist = std::sqrt(wx*wx + wy*wy + wz*wz);
            {
                static auto last_dbg = std::chrono::steady_clock::now();
                auto now_dbg = std::chrono::steady_clock::now();
                if (dist < 5.0 && std::chrono::duration_cast<std::chrono::milliseconds>(now_dbg - last_dbg).count() > 1000) {
                    last_dbg = now_dbg;
                    fprintf(stderr, "[OBB-DBG] dist=%.2f  quat=(%.4f,%.4f,%.4f,%.4f)  world_d=(%.3f,%.3f,%.3f)  local_d=(%.3f,%.3f,%.3f)  bbox=(%.3f,%.3f,%.3f)  %s\n",
                            dist, tgt_qw, tgt_qx, tgt_qy, tgt_qz,
                            wx, wy, wz, lx, ly, lz,
                            g_target_bbox_x, g_target_bbox_y, g_target_bbox_z,
                            inside ? "INSIDE" : "outside");
                }
            }

            if (inside && !was_inside) {
                fprintf(stderr, "[gz_image_bridge] TARGET REACHED! drone=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f) local=(%.2f,%.2f,%.2f)\n",
                        drone_x, drone_y, drone_z, tgt_x, tgt_y, tgt_z, lx, ly, lz);
                g_target_reached.store(true, std::memory_order_release);
            }
        }
    }

    for (int i = 0; i < _msg.pose_size(); i++)
    {
        const auto &p = _msg.pose(i);
        if (p.name() != g_model_name) continue;

        double x  = p.position().x();
        double y  = p.position().y();
        double z  = p.position().z();
        double qw = p.orientation().w();
        double qx = p.orientation().x();
        double qy = p.orientation().y();
        double qz = p.orientation().z();

        // Store drone world pose for OSD bearing indicator
        {
            double yaw = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
            std::lock_guard<std::mutex> lk(g_drone_mutex);
            g_drone_x = x; g_drone_y = y; g_drone_z = z;
            g_drone_yaw = yaw;
            g_drone_valid = true;
        }

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
            ;  // accepted for backward compat, OSD is on by default
        else if (strcmp(argv[i], "--no-osd") == 0)
            g_osd_enabled = false;
        else if (strcmp(argv[i], "--thermal") == 0)
            g_thermal = true;   // white-hot grayscale styling
        else if (strcmp(argv[i], "--mavlink-osd") == 0)
        {
            g_mavlink_osd  = true;
            g_osd_enabled  = true;  // override --no-osd if both given
        }
        else if (strcmp(argv[i], "--mavlink-port") == 0 && i + 1 < argc)
            g_mavlink_port = atoi(argv[++i]);
        else if (strcmp(argv[i], "--msp-port") == 0 && i + 1 < argc)
            g_msp_port = atoi(argv[++i]);
        else if (strcmp(argv[i], "--stream") == 0 && i + 1 < argc)
            { g_stream_dest = argv[++i]; g_stream_rtsp = false; }
        else if (strcmp(argv[i], "--rtsp") == 0 && i + 1 < argc)
            { g_stream_dest = argv[++i]; g_stream_rtsp = true; }
        else if (strcmp(argv[i], "--stream-fps") == 0 && i + 1 < argc)
            g_stream_fps = std::max(1, atoi(argv[++i]));
        else if (strcmp(argv[i], "--stream-width") == 0 && i + 1 < argc)
            g_stream_width = std::max(0, atoi(argv[++i]));
        else if (strcmp(argv[i], "--stream-height") == 0 && i + 1 < argc)
            g_stream_height = std::max(0, atoi(argv[++i]));
        else if (strcmp(argv[i], "--stream-bitrate") == 0 && i + 1 < argc)
            g_stream_bitrate = argv[++i];
        else if (strcmp(argv[i], "--stream-preset") == 0 && i + 1 < argc)
            g_stream_preset = argv[++i];
        else if (strcmp(argv[i], "--stream-tune") == 0 && i + 1 < argc)
            g_stream_tune = argv[++i];
        else if (strcmp(argv[i], "--cam-pitch") == 0 && i + 1 < argc)
            g_cam_pitch_deg = atof(argv[++i]);
        else if (strcmp(argv[i], "--out-width") == 0 && i + 1 < argc)
            g_out_width = static_cast<uint32_t>(std::max(64, atoi(argv[++i])));
        else if (strcmp(argv[i], "--out-height") == 0 && i + 1 < argc)
            g_out_height = static_cast<uint32_t>(std::max(64, atoi(argv[++i])));
        else if (strcmp(argv[i], "--display") == 0)
            g_display_mode = true;
        else if (strcmp(argv[i], "--hidden") == 0)
            g_hidden_mode = true;
        else if (strcmp(argv[i], "--target-model") == 0 && i + 1 < argc)
            g_target_model = argv[++i];
        else if (strcmp(argv[i], "--target-link") == 0 && i + 1 < argc)
            g_target_link = argv[++i];
        else if (strcmp(argv[i], "--target-bbox") == 0 && i + 1 < argc) {
            // Parse "X,Y,Z" half-extents
            char *bbox_str = argv[++i];
            if (sscanf(bbox_str, "%lf,%lf,%lf", &g_target_bbox_x, &g_target_bbox_y, &g_target_bbox_z) != 3)
                fprintf(stderr, "[gz_image_bridge] Warning: --target-bbox expects X,Y,Z (got '%s')\n", bbox_str);
        }
        else if (strcmp(argv[i], "--hit-box-scale") == 0 && i + 1 < argc)
            g_hit_box_scale = atof(argv[++i]);
        else if (topic.empty())
            topic = argv[i];
    }

    if (topic.empty())
    {
        fprintf(stderr,
            "Usage: %s <image_topic> [--msp-port PORT]\n"
            "  --msp-port N       MSP TCP port (default: 5763 = UART3)\n"
            "  --mavlink-osd      Use MAVLink UDP telemetry (PX4 stack) instead of MSP\n"
            "  --mavlink-port N   MAVLink UDP port (default: 14550)\n"
            "  --stream H:P       Stream raw (no OSD) H.264 mpegts over UDP to host:port\n"
            "  --rtsp URL         Push raw (no OSD) H.264 to an RTSP server (e.g.\n"
            "                     rtsp://127.0.0.1:8554/tracker)\n"
            "  --stream-fps N     Stream encoder framerate (default: 30)\n"
            "  --stream-width N   Explicit stream output width (0=camera width)\n"
            "  --stream-height N  Explicit stream output height (0=camera height)\n"
            "  --stream-bitrate V libx264 target bitrate, e.g. 4M (default: 4M)\n"
            "  --stream-preset P  libx264 -preset (default: ultrafast)\n"
            "  --stream-tune T    libx264 -tune (default: zerolatency)\n"
            "  --cam-pitch DEG    Camera pitch in degrees (default: -80)\n"
            "  --out-width PX     Output frame width after stretch (default: 640)\n"
            "  --out-height PX    Output frame height after stretch (default: 480)\n"
            "  --display          Render in SDL2 window (zero-latency, no stdout)\n"
            "  --hidden           With --display: create SDL2 window hidden (SHM still active)\n"
            "  --no-osd           Disable OSD overlay and OSD shared memory segment\n"
            "  --thermal          White-hot grayscale styling (simulated thermal cam)\n"
            "  --target-model N   SDF model name of the target (enables proximity detection)\n"
            "  --target-link L    Link within the model to track (for multi-link models)\n"
            "  --target-bbox X,Y,Z  Half-extents in metres (default: 0.792,1.047,0.186)\n"
            "  --hit-box-scale S  Uniform scale for hit box (default: 1.0)\n",
            argv[0]);
        return 1;
    }

    std::signal(SIGINT, sigHandler);
    std::signal(SIGTERM, sigHandler);
    // Writing to a dead ffmpeg pusher's pipe must NOT kill us — handle it as an
    // EPIPE write error (streamWriteFrame) so the stream can be respawned.
    std::signal(SIGPIPE, SIG_IGN);

    // Start telemetry thread (MSP for Betaflight, MAVLink UDP for PX4)
    std::thread osd_thread;
    if (g_osd_enabled) {
        if (g_mavlink_osd) {
            fprintf(stderr, "[gz_image_bridge] MAVLink OSD enabled — UDP port %d\n", g_mavlink_port);
            osd_thread = std::thread(mavlinkThread);
        } else {
            fprintf(stderr, "[gz_image_bridge] OSD enabled — MSP port %d\n", g_msp_port);
            osd_thread = std::thread(mspThread);
        }
    } else {
        fprintf(stderr, "[gz_image_bridge] OSD disabled\n");
    }

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
            if (!g_target_model.empty()) {
                fprintf(stderr, "[gz_image_bridge] Target proximity: '%s'%s%s bbox=(%.3f,%.3f,%.3f) scale=%.2f\n",
                        g_target_model.c_str(),
                        g_target_link.empty() ? "" : "::",
                        g_target_link.c_str(),
                        g_target_bbox_x, g_target_bbox_y, g_target_bbox_z, g_hit_box_scale);
            }
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
            fprintf(stderr, "IMGMETA %u %u %s\n", g_out_width, g_out_height, g_pix_fmt);
            fflush(stderr);
            meta_printed = true;

            // Determine channels from pixel format
            if (strcmp(g_pix_fmt, "rgba") == 0 || strcmp(g_pix_fmt, "bgra") == 0)
                ch_count = 4;
            else
                ch_count = 3;

            // (ffmpeg stream child is spawned — and respawned on death — by
            //  streamWriterThread once g_meta_ready is set.)

#ifdef HAS_SDL2
            // Initialize SDL2 display once we know the frame dimensions
            if (g_display_mode) {
                if (!initDisplay(g_out_width, g_out_height, g_pix_fmt)) {
                    fprintf(stderr, "[display] Failed to init — falling back to stdout\n");
                    g_display_mode = false;
                }
            }
#endif

            // Initialize POSIX shared memory (always active)
            {
                std::string base = shmNameFromTopic(topic);
                g_shm_clean.name = base;
                if (!initShmSegment(g_shm_clean, g_out_width, g_out_height, ch_count, g_pix_fmt))
                    fprintf(stderr, "[shm] Failed to initialize clean segment\n");
                if (g_osd_enabled) {
                    g_shm_osd.name = base + "_osd";
                    if (!initShmSegment(g_shm_osd, g_out_width, g_out_height, ch_count, g_pix_fmt))
                        fprintf(stderr, "[shm] Failed to initialize OSD segment\n");
                }
            }
        }

        if (meta_printed) {
            frame = stretchFrameNearest(
                frame,
                g_width,
                g_height,
                g_out_width,
                g_out_height,
                ch_count
            );
        }

        // White-hot thermal styling (dedicated thermal-cam instance). Applied
        // before SHM/stream/OSD so every consumer sees the thermal frame.
        if (g_thermal && meta_printed)
            applyThermalWhiteHot(frame, g_out_width, g_out_height, ch_count, g_pix_fmt);

        // ── Clean SHM (always, pre-OSD frame for CV/tracker consumers) ──
        if (meta_printed)
            shmSegmentWrite(g_shm_clean, frame);

        // ── Raw LAN stream (before OSD so frames are clean) ──
        // Hand off to the dedicated stream writer thread (non-blocking). Feed it
        // whenever streaming is configured — even while the ffmpeg pusher is down
        // — so the thread can (re)spawn it (e.g. RTSP server started late).
        if (!g_stream_dest.empty())
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
            if (g_mavlink_osd)
                renderPx4Osd(pixels, static_cast<int>(g_out_width),
                             static_cast<int>(g_out_height), ch_count);
            else
                renderOsd(pixels, static_cast<int>(g_out_width),
                          static_cast<int>(g_out_height), ch_count);
            // OSD SHM — post-OSD frame for display consumers
            shmSegmentWrite(g_shm_osd, frame);
        }

#ifdef HAS_SDL2
        // ── Direct display: upload texture and present (sub-millisecond) ──
        if (g_display_mode && g_sdl_texture)
        {
            displayFrame(reinterpret_cast<const uint8_t*>(frame.data()),
                         g_out_width, ch_count);
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
