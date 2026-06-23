# CLAUDE.md — Agent Handover (`aeroloop_gazebo/`)

Gazebo Harmonic worlds, models, and C++ plugins for the drone SITL visualizer.

## Plugins (`plugins/`)

| File | Purpose |
|------|---------|
| `gz_image_bridge.cc` | Camera image bridge → SDL2 display + POSIX shm + optional H.264 stream, **with native OSD overlay** |
| `ExternalPosePlugin.cc` | Receives `VisualPosePacket` over UDP, sets entity `WorldPoseCmd` (drives drone/target poses) |
| `BetaflightPlugin.cc` | In-Gazebo BF physics backend (when not using Simulink) |
| `ViscousDragPlugin.cc` | Aerodynamic drag |
| `RotorVisualPlugin.cc` | Spinning rotor visuals |
| `osd_font.h` | Embedded 8x8 IBM VGA/CP437 bitmap font |

Build: `cd plugins/build && cmake .. && make` (or `build_plugin.sh`). Needs
`libsdl2-dev`; links `pthread`, `rt`, SDL2.

> **Build gotcha:** `gz_image_bridge.cc` is large and pulls in heavy gz-msgs
> protobuf headers; a parallel `make` can be **OOM-killed (exit 137 / "Killed")**
> in constrained environments. If that happens, build single-job: `make -j1
> gz_image_bridge`. Only `gz_image_bridge.cc` is affected (it's the big TU).

## gz_image_bridge OSD — two telemetry sources

OSD is rendered **natively in C++**, no Python in the data path. The bridge
composites OSD onto raw frames before SDL2/shm/stream output.

- **BF stack:** background thread speaks **MSP V1 over TCP** (UART2 = TCP 5762),
  polls at 10 Hz.
- **PX4 stack:** `mavlinkThread()` parses **MAVLink over UDP** (default port
  **14560**): HEARTBEAT (arm/mode), ATTITUDE (roll/pitch/yaw), VFR_HUD
  (throttle/alt/airspeed/climb). `renderPx4Osd()` draws it.

CLI: `--target-model`, `--target-link`, `--target-bbox`, `--mavlink-port`,
`--display`, `--shm`, `--stream host:port`, OSD flags.

## OSD layout model (`renderOsd` / `renderPx4Osd`)

There are **two** render functions that must be kept in **parity**: `renderOsd()`
(BF/MSP) and `renderPx4Osd()` (PX4/MAVLink). Any layout change to one almost
always needs the same change in the other.

- Everything is drawn with `drawElem(frame, fw, fh, ch_count, x, y, text, scale,
  r,g,b)` (dark background + shadowed glyphs). Positions are computed from a few
  locals, not hard-coded pixels:
  - `scale`: `fw>=1280 → 2`, `fw>=640 → 2`, else `1`. (This is the "~75%" size —
    the original used `3 / 2 / 1`; dropping the 1280 tier to `2` shrinks text on
    HD frames.)
  - `cw = ch = 8*scale` (glyph cell), `margin = 8*scale` (≈2× the original `4*scale`,
    which pulls every corner element inward off the frame edge).
- Corner anchoring pattern (right/bottom elements subtract their own pixel width
  from `fw`/`fh`): `fw - margin - strlen(text)*cw` for right-aligned,
  `fh - margin - ch*N - pad` for stacked bottom rows.
- Current placement: **top-left** = roll / pitch / **HDG** stack; **top-right** =
  THR / timer (BF derives THR from avg active motor PWM; PX4 uses VFR_HUD
  `throttle_pct`); **bottom-center** = flight-mode (DISARMED/ANGLE/HORIZON/ACRO,
  or ARMED/DISARMED for PX4) + CH6 guidance (MANUAL/TERMINAL); **bottom-left** =
  FWD / ALT(+climb arrow) / VS; **bottom-right** = heading N/NE/…; **right-center**
  = variometer bar; **center** = body-Z crosshair + artificial horizon.
- `HDG:` lives in the top-left stack (row 3 under roll/pitch), **not** bottom-center.
  Status labels (flight mode + guidance) sit bottom-center where HDG used to be.
- Guidance/lock labels read RC channels straight from telemetry: CH6 (`channels[5]
  > 2000`) = TERMINAL, CH9 (`channels[8] > 2000`) = LOCK. These only exist in the
  BF (`renderOsd`) path since PX4 has no MSP RC array.

## Target tracking (`onPoseV`)

- `g_target_model` must be set via `--target-model`, else target tracking is
  skipped entirely.
- `dynamic_pose/info` publishes **bare** link names and only entities whose pose
  **changed** — cache model + link poses in **static** variables (they may arrive
  in different `Pose_V` messages). Compose: `link_world = model_pos + R(model_q)*link_rel`.
- Multi-link targets (park_chase): root is fixed at orbit center; visual body is
  `geranium_link` at the end of a revolute chain.

## Recent Fixes (CRITICAL — keep these invariants)

### 1. MAVLink v2 truncation (THR/ALT/VS stuck at 0)
MAVLink v2 truncates trailing zero bytes, so e.g. a VFR_HUD payload arrives
shorter than the 20-byte struct and an `if (payload_len >= 20)` check failed.
**Fix:** zero-fill a local struct-sized buffer, then `memcpy` only `min(payload_len,
sizeof)` bytes before reading fields. Applied to HEARTBEAT, ATTITUDE, and VFR_HUD.

### 2. stderr / mutex deadlock (cameras froze on disarm/land)
The MAVLink thread did per-packet `fprintf(stderr, …)` **inside** the
`g_telem_mutex` scope. The parent Python launcher had `stderr=PIPE` and stopped
draining after IMGMETA → 64 KB pipe filled → `fprintf` blocked while holding the
mutex → render loop deadlocked acquiring it. **Fixes:**
- Removed all per-packet debug `fprintf`.
- Scoped `g_telem_mutex` to **data updates only** — never do I/O under the lock.
- (Launcher side) `betaloop/common.py` drains the bridge's stderr in a daemon thread.

**Invariant:** never block a bridge thread on I/O while holding `g_telem_mutex`.

## Worlds & balloon target

`worlds/rocket_drone_balloon_test_vis.sdf` contains `<model name="balloon_target">`
with `ExternalPosePlugin` listening on UDP **9014**. The pose is driven by
`betaloop/common.start_balloon_thread()` (Lissajous drift at 60 Hz). SDFs are
rendered from Jinja2 templates (see repo memory `jinja2_sdf_templates`).

## Gazebo SDF gotchas (verified)

- **Velocity-controlled joints ignore position limits without an effort limit.**
  A prismatic/revolute joint driven by `JointController` (`<initial_velocity>` or
  `cmd_vel`) will sail straight through its `<lower>`/`<upper>` limits unless the
  same `<limit>` block also sets `<effort>` (use ~`1e6`). Symptom in the log:
  `[JointFeatures.cc:175] Velocity control does not respect positional limits of
  joints if these joints do not have an effort limit.` Add `<effort>1e6</effort>`
  alongside `<lower>`/`<upper>`.
- **`<sky><clouds>` only animate with motion params.** Clouds look frozen
  relative to the drone unless `<clouds>` has a non-trivial `<speed>` (we use
  ~12) and a `<direction>`; add a `<sky><time>` too. Low/default `<speed>` reads
  as a static skybox.

## Target reference dimensions

- **Shahed mesh** (`models/shahed_drone/meshes/shahed.glb`): raw bbox
  1.436 × 0.337 × 1.898 m in mesh space. The model.sdf applies a `-1.5708 0 0`
  X-rotation, so in world frame it is **X (length) 1.44 m, Y (wingspan) 1.90 m,
  Z (height) 0.34 m**. The `target_bbox` half-extents `0.792,1.047,0.186` used in
  the launcher `WORLD_MAP`s are ~half of these.

## OSD target bearing indicator (screen convention)

The FPV/target bearing pointer uses **screen** convention, not math convention:
0° relative bearing (target dead ahead) points **UP**. Pointer endpoint from
screen center is `px = cx - sin(rel)`, `py = cy - cos(rel)` (note the `-cos` for
`y`, and `sin` on `x`), and the 8-way arrow index uses **negated** `rel_rad` so
CCW/left maps to `<`. Using the naive `cos`/`sin` (math) convention puts the
pointer 90° off and swaps left/right.

## Session Addendum (2026-06-13)

- No plugin source changes in this session, but BF default control mapping changed
  upstream: ANGLE mode now maps to CH11/AUX7 (`aux ... 6 ...`) in
  `configure_betaflight.py`.
- Practical effect for OSD/flight testing: if the radio/sim pipeline still drives
  CH10 for ANGLE, expected mode transitions will not occur; align control mapping
  to CH11 when validating BF mode-dependent overlays/behavior.

## `target_chase_cam.sdf.j2` — UDP-driven chase-camera world

Rendered by `target_chase/start.py` (standalone tool, see root `CLAUDE.md`). Key
plugin/world facts learned building it:

- **`ExternalPosePlugin` is multi-instance via `<listen_port>`.** Each model gets
  its own listener: `target_geranium` on **9020**, `chase_camera_rig` on **9021**.
  The plugin defaults to 9010 if `<listen_port>` is omitted. The wire format is the
  same 72-byte `VisualPosePacket` (`<Qd3d4d`: `seq` u64, `t` f64, `pos_enu[3]` f64,
  `quat_wxyz[4]` f64). Python packs it with `struct` and `sendto` per model.
- **No joints for motion.** The earlier prismatic-joint + `JointController` +
  `gz topic -p cmd_vel` design had 1\u20132 s latency (subprocess spawn per command).
  Driving model poses directly over UDP via `ExternalPosePlugin` is sub-ms. Prefer
  this for any externally scripted motion; reserve joints for real physics.
- **Camera optical axis = model +X.** The rig model is yawed `+1.5708` in its base
  `<pose>` so +X points along the orbit tangent; Python then sends `yaw = theta +
  pi/2`. A small upward pitch (`--cam-pitch`, default **5.7\u00b0**) reduces ground in
  frame.
- **Pitch shifts the visible center \u2014 frame bounds must follow.** When the camera
  is pitched, the on-screen center of the orbit plane moves by `depth * tan(pitch)`.
  The Python slide clamp uses `v_center = depth*tan(pitch)` so the target stays
  centered in the *visible* frame, not the geometric one. Frame half-extents also
  scale with actual depth: `frame_half = (chase_distance + pos_d) * tan(fov/2) *
  margin`.
- **Camera res is 1280\u00d7960 (4:3).** `vfov = hfov * 960/1280`. If you change the SDF
  `<image>` size, update the `vfov` ratio in `start.py` (two places) to match, or
  the dynamic frame scaling skews.

## Joystick input (`/dev/input/js*`)

`target_chase/start.py` reads the Linux joydev directly (no SDL/pygame): 8-byte
events `struct <IhBB` = `time` u32, `value` i16, `type` u8, `number` u8. Notes:

- On open, the kernel replays all axes/buttons with the `0x80` (`JS_INIT`) bit set
  \u2014 skip those. `type & 0x02` = axis event; value is \u00b132767.
- Axis map used: **0** = left/right, **1** = up/down, **5** = depth (toward/away
  from camera). Some axes rest off-center (e.g. an axis reading ~\u221233% at idle), so
  a per-axis dead-zone is needed; the depth axis uses a smaller dead-zone for
  responsiveness. Sign flips are common per stick \u2014 negate `value/32767` as needed.

## Session Addendum (2026-06-22)

- Physics `.world(.j2)` files + physics model SDFs (rocket_drone, thaqib_1_prototype,
  orphans) **deleted** — only `_vis.sdf.j2` worlds remain (shared mesh dirs kept).
- Vis-model cameras: `fpv_tracker_cam`→`fpv_tracker_wide_cam` (now
  `type="wideanglecamera"` + fisheye `<lens>`); new `fpv_tracker_narrow_cam` +
  `fpv_thermal_cam`; all five sensors wrapped in `{% if *_enabled %}`. narrow +
  thermal are plain `camera` (rectilinear) — fisheye cubemaps (6 faces each)
  serialise the single gz render thread and tank RTF; only the wide tracker is
  fisheye. env_texture_size 512 is the wide cubemap's remaining cost knob.
- Selectable target: world target visuals use `{{ target_mesh_uri }}` (park_chase,
  patrol_park) / `{{ target_model_uri }}` include (collision_test); new
  `models/stingjet/` (glb only). Vars come from `betaloop` `TARGET_REFS`.
- Perf: `baylands_terrain` visual `cast_shadows=false` (the terrain was the whole
  shadow-map cost); scene `<shadows>` stays true so the target still casts.

## Session Addendum (2026-06-23) — per-camera fisheye toggle

- Each vis model template's wide/narrow/thermal `<sensor>` switches projection on a
  `*_fisheye` var: `type="{{ 'wideanglecamera' if <cam>_fisheye else 'camera' }}"`,
  and the fisheye `<lens>` block (`env_texture_size` 512) is wrapped in
  `{%- if <cam>_fisheye %}`. Vars `tracker_wide_fisheye` (default true) /
  `tracker_narrow_fisheye` / `thermal_fisheye` (default false) come from `betaloop`
  `compute_model_vars`. Both projections render to valid XML for all 3 templates.
  Narrow/thermal default rectilinear (fisheye cubemaps = 6 render passes each,
  serialise the single gz render thread).

## Session Addendum (2026-06-23) — utility camera sensor

- Each vis model template gained a `fpv_utility_cam` `<sensor>` (cloned from the wide
  tracker, gated by `{%- if utility_cam_enabled %}`) at the wide→narrow boundary. Same
  fisheye-vs-rectilinear switch as the others
  (`type="{{ 'wideanglecamera' if utility_fisheye else 'camera' }}"` + conditional
  `<lens>`). Mount matches the template's wide tracker (rocket/thaqib `0 0 0.262`,
  iris `0.15 0 0.03`). Vars `utility_*` come from `betaloop` `compute_model_vars`.
