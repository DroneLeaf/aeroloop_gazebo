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
- **OSD overlay** (always enabled) — composites FPV-style telemetry (battery, attitude,
  altitude, flight mode, timer) by querying Betaflight SITL via MSP over TCP.
- **Target proximity detection** — when `--target-model` is set, monitors the drone's
  distance to the named target via the Gazebo `dynamic_pose/info` topic. Displays
  a flashing "TARGET REACHED" OSD indicator when the drone enters the target's
  oriented bounding box (OBB). The indicator latches until world reset.
  Use `--hit-box-scale` to uniformly enlarge/shrink the hit box.

```
Usage: gz_image_bridge <image_topic> [options]
  --msp-port N           MSP TCP port (default: 5763 = UART3)
  --stream H:P           Stream H.264 over UDP to host:port
  --cam-pitch DEG        Camera pitch in degrees (default: -80)
  --display              Render in SDL2 window (zero-latency)
  --hidden               With --display: create window hidden (SHM still active)
  --no-osd               Disable OSD overlay
  --target-model NAME    SDF model name of the target (enables proximity detection)
  --target-bbox X,Y,Z    Half-extents in metres (default: 0.792,1.047,0.186)
  --hit-box-scale S      Uniform scale for hit box (default: 1.0)
```

First frame metadata is printed to stderr: `IMGMETA <width> <height> <pix_fmt>`

## Build

```bash
mkdir build
cd build
cmake ..
make
```
