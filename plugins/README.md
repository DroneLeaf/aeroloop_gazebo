# Gazebo Plugins

## BetaflightPlugin

Gazebo system plugin that bridges motor commands from Betaflight SITL (UDP 9002)
to model joint forces and returns FDM state (UDP 9003).

## gz_image_bridge

Subscribes to a Gazebo camera image topic via gz-transport and renders
frames in an SDL2 window (`--display`), optionally exposing them via POSIX
shared memory (`--shm`).

Features:
- **Single-slot latest-frame buffer** — never accumulates latency; old frames are
  silently dropped if the downstream consumer can't keep up.
- **OSD overlay** (`--osd`) — composites FPV-style telemetry (battery, attitude,
  altitude, flight mode, timer) by querying Betaflight SITL via MSP over TCP.

```
Usage: gz_image_bridge <image_topic> [--osd [--msp-port PORT]]
```

First frame metadata is printed to stderr: `IMGMETA <width> <height> <pix_fmt>`

## Build

```bash
mkdir build
cd build
cmake ..
make
```
