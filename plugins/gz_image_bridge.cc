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
 *   gz_image_bridge <image_topic>
 *
 * First frame metadata is printed to stderr:
 *   IMGMETA <width> <height> <pix_fmt>
 *
 * pix_fmt values match ffmpeg names: rgb24, rgba, bgr24, bgra
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

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

static void sigHandler(int) { g_running = false; g_cv.notify_all(); }

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

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        fprintf(stderr, "Usage: %s <image_topic>\n", argv[0]);
        return 1;
    }

    std::signal(SIGINT, sigHandler);
    std::signal(SIGTERM, sigHandler);

    const std::string topic = argv[1];

    gz::transport::Node node;
    if (!node.Subscribe(topic, onImage))
    {
        fprintf(stderr, "[gz_image_bridge] Failed to subscribe to %s\n",
                topic.c_str());
        return 1;
    }

    fprintf(stderr, "[gz_image_bridge] Subscribed to %s — waiting for frames\n",
            topic.c_str());

    bool meta_printed = false;

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
    return 0;
}
