/*
 * gz_image_bridge - Subscribe to a Gazebo image topic and write raw frames
 *                   to stdout for piping into ffmpeg or other consumers.
 *
 * Usage:
 *   gz_image_bridge <image_topic>
 *
 * First frame metadata is printed to stderr:
 *   IMGMETA <width> <height> <pix_fmt>
 *
 * pix_fmt values match ffmpeg names: rgb24, rgba, bgr24, bgra
 *
 * Example:
 *   gz_image_bridge /world/fpv_demo/model/.../sensor/camera/image \
 *     | ffmpeg -f rawvideo -pix_fmt rgb24 -s 640x480 -r 30 -i pipe:0 \
 *       -c:v libx264 -preset ultrafast -tune zerolatency \
 *       -f mpegts "tcp://0.0.0.0:8554?listen=1"
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

static std::atomic<bool> g_running{true};
static std::atomic<bool> g_got_first{false};

static void sigHandler(int) { g_running = false; }

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

static void onImage(const gz::msgs::Image &_msg)
{
    if (!g_got_first.exchange(true))
    {
        const char *fmt = pixelFormatStr(_msg.pixel_format_type());
        fprintf(stderr, "IMGMETA %u %u %s\n", _msg.width(), _msg.height(), fmt);
        fflush(stderr);
    }

    const std::string &data = _msg.data();
    if (!data.empty() && g_running)
    {
        size_t written = fwrite(data.data(), 1, data.size(), stdout);
        if (written != data.size())
        {
            // Pipe closed (ffmpeg exited, etc.)
            g_running = false;
            return;
        }
        fflush(stdout);
    }
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

    while (g_running)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    fprintf(stderr, "[gz_image_bridge] Shutting down\n");
    return 0;
}
