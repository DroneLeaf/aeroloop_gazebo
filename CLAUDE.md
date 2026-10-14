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
| `ShmCameraExportPlugin.cc` | In-process camera → POSIX shm export (`/gz_cam_<model>_<sensor>_raw`); pairs with `gz_image_bridge --shm-source` so gz never serialises image frames |
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

## Session Addendum (2026-06-23) — parameterised target scale (all targets)

- Both `models/stingjet/model.sdf` and `models/shahed_drone/model.sdf` are now
  generated from `model.sdf.j2` (`<scale>{{ target_scale | default(...) }}</scale>`
  on visual + collision; defaults `0.1 0.1 0.1` stingjet / `1 1 1` shahed), rendered
  by `betaloop` render_vis_templates and **gitignored** (like the `*_vis.sdf`
  models). The inline worlds (park_chase/patrol_park) already used
  `{{ target_scale }}`; collision_test (which `<include>`s `model://<target>`) now
  picks up the scale via the rendered model.sdf. Keep XML comments free of `--`
  (strict parsers reject it; Gazebo tolerates).

## Session Addendum (2026-06-23) — moving_target world (park_chase + patrol_park collapse)

- `worlds/rocket_drone_park_chase_vis.sdf.j2` + `..._patrol_park_vis.sdf.j2`
  **deleted**; replaced by `worlds/rocket_drone_moving_target_vis.sdf.j2`
  (`<world name="fpv_moving_target">`, target model `moving_target` on
  ExternalPosePlugin 9016). The target spawns at `{{ target_spawn_x/y }}` /
  `{{ target_spawn_yaw }}` (trajectory s=0); player drone yaw is
  `{{ player_heading_rad }}` (configurable, default 0 = east). Light attenuation
  range bumped to 20000 (the larger ex-patrol value) for the 2–3 km trajectory area.

## Session Addendum (2026-06-24) — RTSP CRF rate control (fixes 480p "corruption")

- **Root cause of RTSP artifacts at higher res:** NOT a malformed bitstream and
  NOT CPU saturation. The raw-frame → ffmpeg pipe path is provably clean
  (`streamWriteFrame` is a complete blocking write; frame bytes ==
  `-video_size g_out_width×g_out_height×ch_count`; `streamWriterThread` swaps the
  shared buffer to a thread-local under the lock before the write; spawn dims match).
  A lossless TCP transport carrying artifacts only proves *publisher-side*, which is
  equally true of heavy quantization. The artifacts were **bitrate starvation**: a
  fixed `-b:v 4M` spread over ~4× the pixels at 480p → ~0.08–0.33 bits/px → macroblocking.
- **Fix:** `--stream-crf N` (global `g_stream_crf`, default -1 = off). When N>=0,
  `spawnStreamFfmpeg` emits **capped CRF** `-crf N -maxrate <bitrate> -bufsize <bitrate>`
  (quality-targeted, bitrate as a ceiling) instead of `-b:v`. CRF 23 at 480p draws
  ~4–6 Mbps (under the 8 Mbps default cap) → no starvation, full detail kept. Plumbed
  as `--<feed>-rtsp-crf` (betaloop) → `--stream-crf` (bridge); UI "Quality (CRF)"
  spin per RTSP card (0 = off → ABR bitrate). Default 23 everywhere (UI/supervisor
  emit it even when unset, so the fix is on by default).

## Session Addendum (2026-06-24) — per-camera fisheye lens intrinsics

- Each vis-model template's fisheye `<lens>` `<custom_function>` is now templated:
  `<c1>{{ <cam>_lens_c1 }}</c1><c2>{{ <cam>_lens_c2 }}</c2><c3>{{ <cam>_lens_c3 }}</c3>
  <f>1.0</f><fun>{{ <cam>_lens_fun }}</fun>` (added an explicit `<c3>`; `scale_to_hfov`
  + `cutoff_angle 3.1415` + `env_texture_size 512` unchanged). Vars `<cam>_lens_c1/c2/c3/fun`
  for `tracker_wide` / `tracker_narrow` / `thermal` / `utility` come from `betaloop`
  `compute_model_vars` (defaults 1.05/4.0/0.0/tan → byte-identical render to before).
  Mapping: `r = c1*f*fun(theta/c2 + c3)`; `fun` ∈ {tan,sin,id}. All 3 templates
  (rocket_drone/thaqib/iris) edited identically.

## Session Addendum (2026-06-28) — balloon target + windy_target world

- New **`models/balloon/`** (`model.sdf.j2` + `model.config`): a primitive sphere
  target (link `body`, scalable `target_radius`/`target_color`), rendered to a
  gitignored `model.sdf` like the other targets. Used by worlds that `<include>`
  the target (collision_test). Added to `.gitignore`.
- World template **`rocket_drone_balloon_test_vis.sdf.j2` → `rocket_drone_windy_target_vis.sdf.j2`**
  (`git mv`; `<world name="windy_target">`). The hardcoded red-sphere `balloon_target`
  model became a generic `windy_target` model whose visual branches
  `{% if target_primitive == 'sphere' %}<sphere>{% else %}<mesh>{% endif %}` (still
  ExternalPosePlugin UDP 9014, link `body`).
- `rocket_drone_moving_target_vis.sdf.j2` gained the same sphere/mesh branch on
  `geranium_link` (so the balloon works as a moving target). `rocket_drone_shake_test_vis.sdf.j2`
  gained a **static** `shake_target` model (same branch) at `target_x/y/z`.
- The branch keys come from `betaloop.compute_world_vars`: `target_primitive`
  ('sphere'|''), `target_radius`, `target_color` (mesh path stays `target_mesh_uri`
  + `target_visual_pose` + `target_scale`).

## Session Addendum (2026-07-16) — terrain themes + sky/sun templating (moving_target)

- `models/baylands_terrain/media/Textures/themes/{desert,lush}/` hold per-theme
  copies of the three ground textures the DAE references (Grass/Sand/DirtPath).
  betaloop `apply_terrain_theme` copies a set over the live files at launch —
  the DAE can't be re-pointed per launch (92 MB, hardcoded texture paths).
  `themes/desert/` = snapshot of the stock sandy set; `themes/lush/` is
  regenerable via `themes/generate_lush.py` (PIL; dark green grass from
  `Grass_original.png` + luminance-preserving recolors of sand/dirt).
- `rocket_drone_moving_target_vis.sdf.j2`: `desert_plane` → **`ground_plane`**
  with `{{ ground_color }}`; `<sky><time>`, `<background>`, sun
  diffuse/specular/direction now templated (`sky_time`, `background_color`,
  `sun_diffuse`, `sun_specular`, `sun_direction`). All have `| default(...)`
  equal to the old hardcoded values, so rendering without the vars is unchanged.

## Session Addendum (2026-07-18) — target mesh `<material>` override

- The 3 inline-visual target worlds (`rocket_drone_{moving_target,windy_target,
  shake_test}_vis.sdf.j2`) and both `models/{shahed_drone,stingjet}/model.sdf.j2`
  now emit an optional `<material>` on the target's **visual** when the
  `target_mesh_color` template var is non-empty (from `--target-mesh-color`).
  Empty → no `<material>` element at all, so the mesh keeps its `.glb` material.
- **Confirmed in ogre2 (gz-sim 8):** an SDF `<material>` on a mesh visual fully
  overrides the glTF's embedded PBR material — a red override captured as exactly
  `[170,0,0]` headless, vs the stock shahed mesh's `[110,109,107]`. So this is a
  real override, not a tint that blends with the baked material.
- `<collision>` is deliberately NOT given a material (it has no visual effect and
  would only add noise to the generated SDF).
- **XML-comment gotcha:** `--` is illegal inside an XML comment, so template
  comments must reference the flag as `target-mesh-color`, never
  `--target-mesh-color` — the latter makes the whole generated SDF unparseable.

## Session Addendum (2026-07-18) — stingjet mesh origin re-centred (calibration-critical)

- **Bug:** `models/stingjet/stingjet.glb` (actually an **MQ-9 Reaper** mesh —
  node name `uploads_files_800272_MQ-9.001`, 18.79 m span × 10.69 m length ×
  3.23 m height at scale 1.0) had its geometry **offset from the mesh origin**
  by `(0, 1.36905, 1.04933)` raw units. Because SDF `<scale>` multiplies vertex
  positions **about the mesh origin**, that offset scaled with `target_scale`:
  the visual body sat 0.105 m forward + 0.137 m up from the commanded pose at
  scale 0.1, 0.21/0.27 m at 0.2, 1.05/1.37 m at 1.0. The pose driven over UDP
  (and reported as ground truth) is the MODEL ORIGIN, so the tracked target
  rendered off its own ground-truth position by a **scale-dependent** amount —
  a body-fixed lever arm that rotates with target yaw, so it reads as
  structured noise, not a constant image offset. At the wide tracker
  (1280 px, HFOV 101.8° → f ≈ 520 px) the 0.17 m lever arm at scale 0.1
  projects to ≈9 px at 10 m / 18 px at 5 m.
- **Fix:** the glb's single root node gained `translation = -AABB_centre`
  (JSON-chunk-only edit — vertex/material/UV data untouched, extents provably
  identical). Geometry AABB centre is now exactly `(0,0,0)`, so the mesh stays
  centred at ANY `target_scale` with **no per-scale compensation anywhere**.
  This fixes both consumers at once: the inline-visual worlds
  (moving_target/windy_target/shake_test) and the `<include>` model path
  (collision_test), visual **and** collision.
- **Verified end-to-end**, not assumed: top-down gz render with a marker at the
  commanded pose — silhouette-vs-origin offset went from **−12.0 px (scale 0.1)
  / −30.5 px (0.2)** before, to **+0.50 px / +1.00 px** after, matching the
  perspective-only prediction (+0.32 / +1.26 px) computed by projecting the real
  vertex cloud. (Gotcha while measuring: a `mean<205` silhouette threshold clips
  the thin bright nose tip and fakes a ~4–7 px residual — use `<250`.)
- `shahed.glb` is already centred (1.1 mm ⇒ <0.12 px at 5 m) — no action needed.
- **If the stingjet asset is ever re-downloaded/replaced, the re-centring must be
  re-applied** (add a root-node `translation` of −AABB-centre); otherwise the
  scale-dependent bias silently returns.
- **Still open (flagged, not fixed):** `TARGET_REFS["stingjet"]["bbox"]`
  (`0.94,0.54,0.16`) is ordered X=span, Y=length, which matches the `<include>`
  path (wingspan→body X) but NOT the inline-visual path used by moving_target,
  where the visual pose `1.57079 0 1.5708` maps length→body X, span→body Y
  (true half-extents there: `0.534,0.940,0.161`). Same transposition exists for
  shahed. Check how `gz_image_bridge` orients `--target-bbox` before changing it.

## Session Addendum (2026-07-19) — moving_target far ground is a mesh, not a plane

- `rocket_drone_moving_target_vis.sdf.j2`'s `ground_plane` now renders a
  **UV-tiled 20 km quad** (`{{ ground_mesh_uri }}`, generated by betaloop
  `ensure_ground_mesh`) with `<pbr><metal><albedo_map>{{ ground_texture_uri }}`
  — the terrain's own `Grass.png`, so it tracks `--terrain-theme` and blends
  into the baylands tiles. The old flat-colour `<plane>` survives as an
  `{% if ground_mesh_uri %}` / `{% else %}` fallback.
- **Verified SDF gotcha:** a `<plane>`'s texture is stretched across its whole
  `<size>` — SDF exposes no UV-scale/tiling field, so texturing a 20 km plane
  is pointless. Tiling has to be baked into mesh UVs. Don't "simplify" this
  back to a `<plane>` with an albedo_map.
- **No fog in this stack:** grepped `gz-rendering8` and `sdformat14` — neither
  has any `fog` symbol (`Scene.hh` has no accessor). Atmospheric haze to soften
  the horizon is not reachable from SDF; it needs a custom rendering plugin.
- Quad sits at `z = -0.05` so it never z-fights the terrain (which is lower).

## Session Addendum (2026-07-25) — RTSP encoder: H.264/H.265 selectable

- `spawnStreamFfmpeg` (the ffmpeg child the bridge forks) now honours
  **`--stream-codec {h264,h265,hevc}`** (`g_stream_codec`, default h264):
  `-c:v libx264` vs `libx265`. The codec-params flag switches with it —
  `-x264-params repeat-headers=1` for H.264, `-x265-params
  repeat-headers=1:log-level=error` for H.265 (log-level suppresses x265's
  multi-line banner; repeat-headers keeps SPS/PPS/VPS on every keyframe for
  late RTSP joiners). Preset names are shared across x264/x265; the `-tune`
  guard drops **film/stillimage** under x265 (libx265 hard-errors on them).
- Only the RTSP push path is affected; the raw UDP-mpegts path also picks up
  the codec since it shares the same argv builder.
- **Requires a rebuild** (`cmake --build plugins/build`, or `make -j1
  gz_image_bridge` per the big-TU OOM gotcha). Verified: the exact h265 argv
  ffprobes as `hevc` at the requested size, clean stderr, OpenCV-decodable.

## Session Addendum (2026-08-20) — models/falcon_trainer (generated OBJ target)

- New target `models/falcon_trainer/`: red/black-checkerboard ~1.8 m Falcon
  trainer RC plane. The mesh (`meshes/falcon_trainer.obj` + `.mtl`) is
  **generated lofted-solid geometry, COMMITTED** (an asset like the .glbs;
  regenerable via `generate_falcon.py` — tapered fuselage/wing/stab, rounded
  leading edges from arc-profile ribs, checkers as separate red/black solids,
  wheels 12-gon prisms; per-face winding auto-corrected vs the solid
  centroid). Preview renders `preview_top.png` / `preview_threequarter.png`
  sit in the model dir. Modeled DIRECTLY in the Gazebo body frame (+X nose, +Y span, +Z up,
  AABB centred) so `model.sdf.j2`'s link pose is IDENTITY — unlike
  shahed/stingjet's `-1.5708 0 0`. `model.sdf` is rendered + gitignored like
  the others. Verified in ogre2: OBJ MTL materials ARE honoured (no textures
  involved), and the `target_mesh_color` SDF override fully repaints them.
- Headless capture note: the camera sensor `<save>` element wrote nothing on
  this box; capture frames by subscribing —
  `gz topic -e -n 1 --json-output -t /world/<w>/model/<m>/link/l/sensor/cam/image`
  and base64-decode `data` (set DISPLAY + __EGL_VENDOR_LIBRARY_FILENAMES to
  the NVIDIA glvnd json or EGL context creation fails headless).

## Session Addendum (2026-08-20) — plugins were built at -O0; tracker feed capped at 27 fps

- **Symptom:** leaf-tracker showed 26–27 FPS on a 1920×1080 rectilinear
  wide-tracker feed with only the chase cam also enabled. **Not** the GPU (29%
  busy), **not** the sim (RTF 0.998, the camera published at ~55 Hz): the 1080p
  `gz_image_bridge` ran at 120% CPU and wrote SHM at exactly 27.2 Hz with a
  rock-steady 37 ms per frame — a per-frame CPU cost, dropping every other
  frame through its single-slot latest-wins buffer.
- **Root cause 1 — no optimisation:** `plugins/CMakeLists.txt` set no
  `CMAKE_BUILD_TYPE`, so CMake emitted NO `-O` flag (`flags.make` was just
  `-std=gnu++17`) — every plugin (bridge, ExternalPosePlugin, …) was -O0.
  **Root cause 2 — the per-pixel stretch:** the tracker cam's HFOV 88°/VFOV 47°
  makes `compute_model_vars` render the sensor at **1920×865** (height follows
  the FOV ratio), and the bridge stretches to the requested 1920×1080 on every
  frame; `stretchFrameNearest` did two 64-bit divisions + a 3-byte `memcpy`
  CALL per pixel — **35 ms/frame at -O0** (measured in isolation: 29 fps
  ceiling = the observed cap), 3.7 ms at -O2.
- **Fixes:** (1) CMakeLists now defaults `CMAKE_BUILD_TYPE` to **Release**
  when unset (`-O3 -DNDEBUG`; message "Plugins build type: …" on configure —
  re-run `cmake -S . -B build` once so an old cache picks it up). (2)
  `stretchFrameNearest` is now IN-PLACE (`std::string &frame`, swaps with a
  reused static scratch → zero steady-state allocation, no return copy), one
  row `memcpy` per output row when widths match (the vertical-only case
  above), x-index LUT otherwise — 1.8 ms even at -O0, output byte-identical
  (verified old vs new on 1920×865→1080 and 640×480→1080). Rebuilt all plugins.
- **Verified** in an isolated `GZ_PARTITION` bench world (1920×865 @60 Hz cam,
  same geometry): new bridge SHM rate == source rate (100% of frames kept, 37%
  CPU) vs 27/55 before. Running bridges keep the old binary — **re-Initialize
  to pick up the rebuild.**
- **Fidelity note for operators:** a VFOV that doesn't match the output aspect
  means the tracker is fed NON-SQUARE pixels (865→1080 = 25% vertical
  stretch). For square pixels pick a ratio in the camera card's Aspect Ratio
  combo (16:9 derives VFOV ≈ 57° for HFOV 88°), which also removes the stretch
  work entirely.
- **Harness gotcha:** with `--no-display` the bridge still writes every raw
  frame to STDOUT (legacy ffmpeg-pipe path). The launchers send it to
  `/dev/null` (free); redirecting it to a file/pipe in a test makes the bridge
  disk/pipe-bound (a bench run dropped to 0.5 Hz before this was spotted).

## Session Addendum (2026-09-01) — sensor mount offsets in the drone vis templates

- `models/{rocket_drone,thaqib_1_prototype,iris}_vis/model.sdf.j2`: the
  `fpv_tracker_wide_cam` / `fpv_tracker_narrow_cam` / `fpv_thermal_cam` sensor
  poses now add `{{ <cam>_x|default(0.0) }}` / `{{ <cam>_y|default(0.0) }}`
  (body-FLU METRES from betaloop `compute_model_vars`; the mm + y-right→y-left
  conversion happens there, not here) to the per-drone base mount x/y. The
  iris keeps its 0.15/0.03 base. Zero offsets render byte-identical to before.

## Session Addendum (2026-09-15) — ShmCameraExportPlugin + `gz_image_bridge --shm-source`

- **Why:** gz-sensors publishes camera images over gz-transport (protobuf +
  ZMQ/TCP) FROM THE RENDER THREAD. Measured on the 25-tile baylands world: two
  tracker cams (1188×636 + 926×496 warp sources) render at **91 fps with no
  image subscriber, 45 fps with any** — a null subscriber too. Not the GPU
  (28 %), not the scene, not shadows, not the physics step (all measured).
- **Plugin (model System, `ISystemConfigure` + `ISystemPostUpdate`):**
  enumerates `components::Camera` under its model (optional `<sensor>` filter
  children), computes each camera's due time from `sdf::Sensor::UpdateRate`
  with gz-sensors' catch-up rule, and on `events::PostRender` (render thread)
  does `scene->SensorByName(scoped)` → `Camera::Copy(Image)` → memcpy into
  the shm segment + release fence + `sequence++`. Header = the bridge's
  `ShmHeader` byte-for-byte (`static_assert`ed 64 B in both files). Rendering
  camera name = `scopedName(entity, ecm, "::", false)`; a suffix fallback
  scans `SensorByIndex` (the live name was `world::model::link::sensor`).
- **Bridge:** `--shm-source` derives `shmNameFromTopic(topic) + "_raw"`,
  spawns `shmSourceThread` (mmap, poll `sequence` every 300 µs, seqlock
  read-verify, hands frames to the existing writer loop via `g_frame_data`),
  and skips `node.Subscribe(topic)` — the pose-topic subscription stays. The
  topic arg is still required (naming + pose topic parsing).
- **The plugin RENDERS, it does not just copy:** gz-sensors skips rendering
  any camera without an image-topic subscriber (camera_info subscribers don't
  count — measured), so a copy-only export yields black frames. Per PostRender
  pass: collect due cameras → ONE `Scene::PreRender()` → per camera
  `Camera::Render(); Camera::PostRender()` → ONE `Scene::PostRender()` (the
  gz-sensors pattern; `Camera::Update()` repeats the scene pair per camera and
  the scene PostRender is a GPU flush/new-frame — cost 88→82 fps when doubled).
  betaloop emits the `<plugin>` only in shm transport mode
  (`camera_shm_export`) to avoid double renders.
- **Async readback (the big win, 45→90 fps):** `Camera::Copy()` is a
  synchronous ~450 MB/s staging download (7–9 ms/cam here). Instead:
  `glGetTextureImage(Camera::RenderTextureGLId(), GL_RGBA)` into a 2-deep
  PIXEL_PACK buffer ring + `glFenceSync` + **`glFlush()`** (mandatory — the
  driver otherwise defers the download until the next wait), then on the NEXT
  pass `glClientWaitSync` (≈0 ms) + `glMapBufferRange` + one memcpy of native
  RGBA into the segment. Three traps, all measured: `GL_RGB` requests repack
  synchronously (no gain); no `glFlush` → the fence wait executes the work;
  RGBA→RGB on the render thread costs ~0.8 ms/cam, so the BRIDGE strips alpha
  (`--shm-source` reader: `channels==4` → rgb24). GL entry points via
  `eglGetProcAddress` (dlopen libEGL); PACK_ALIGNMENT/PBO binding saved +
  restored around our calls; `SHM_EXPORT_SYNC=1` forces the Copy path
  (pixel-identical reference for A/B). Stats every 5 s:
  `[ShmCameraExport] <cam>: fps, render, readback [issue, fence-wait,
  map+convert]` + `cycle: between passes / in plugin / outside`.
- **Do NOT hold rendering smart pointers across frames in a system plugin** —
  ogre2 is `dlclose`'d at shutdown and their control blocks then segfault on
  release from the plugin dtor (`Address not mapped` under `~Plugin`). Look up
  per frame; keep only names; build `rendering::Image` via its public ctor.
- **Build:** CMake target `ShmCameraExportPlugin` (links gz-sim8, gz-plugin2,
  gz-rendering8, rt). `find_package(gz-rendering8)` added.

## Session Addendum (2026-09-16) — export plugin: due-flag race fixed, skip detector, shadows templated

- **Race (cost the live 60/79 fps):** `Cam::due` is set on the sim thread
  (PostUpdate, gz-sensors catch-up rule) and was cleared on the render
  thread AFTER the camera rendered — a `due` raised mid-render was lost and
  that camera skipped a whole pass. OnPostRender now consumes the flag when
  it COLLECTS the due cameras (`c->due.store(false)` before any `Render()`),
  so a period that elapses during the pass is picked up by the next one.
  Keep it that way: never clear `due` after rendering.
- **Skip detector:** each camera's 5 s stats line ends with
  `due N / rendered M` (+ ` <-- SKIPPING` when N > M). `dueCount` is bumped
  only on a false→true transition (`due.exchange(true)`), so it counts
  periods, not steps. A healthy run shows N == M at the camera's fps.
- **What contention does (measured, 6 busy cores):** render submission per
  camera 2.0 → 3–4 ms — the ogre2 scene walk + driver work is CPU time on the
  single render thread; the readback stays ~0.6 ms. Two cameras with shadows
  = 12.3 ms/pass > the 11.1 ms step → 80 fps; shadows off = 7.45 ms → 88–89.
- **World templates:** the 5 `rocket_drone_*_vis.sdf.j2` scene blocks carry
  `<shadows>{{ 'true' if scene_shadows | default(true) else 'false' }}
  </shadows>` and the same on the sun's `<cast_shadows>` (template default
  true; betaloop passes False unless `--scene-shadows`). The terrain visual's
  `cast_shadows=false` (2026-06-22) and the target visual's `true` are
  untouched. Template comments name the flag WITHOUT dashes (XML forbids
  `--` in comments — see the 2026-07-18 gotcha). `target_chase_cam.sdf.j2` is
  not templated (its own rig, shadows on).

## Session Addendum (2026-10-03) — multiple moving_target targets

- `rocket_drone_moving_target_vis.sdf.j2`: the target link is a Jinja macro
  `target_link()`; after `moving_target`, a loop over `extra_targets` adds
  `moving_target_<k>` models (own `ExternalPosePlugin` port 9050+k, same
  airframe). Single-target render is XML-equal to before.
- `gz_image_bridge --extra-target-model N` (repeatable): extra targets'
  model poses cached from dynamic_pose/info; TARGET REACHED tests every
  target's OBB (shared hit box), OSD bearing uses the nearest
  (`nearestTarget`). Rebuilt (`cmake --build plugins/build --target
  gz_image_bridge`).

## Session Addendum (2026-10-03b) — ShmCameraExportPlugin: one render pass for every model

- Fleet worlds load one plugin instance per drone model. Instances now
  register in a process-wide registry (`Registry()`/`RegMutex()`); the FIRST
  registered instance leads each PostRender pass: it collects every instance's
  due cameras, brackets them with ONE `Scene::PreRender()`/`PostRender()`, and
  renders/reads them all; other instances' handlers return immediately. The
  mutex is held for the pass, and `~ShmCameraExportPlugin` unregisters under
  it, so teardown can't race a render; a destroyed leader hands over to the
  next instance. Stats lines are now `<model>/<sensor>` and the cycle line
  reports the model count.
- Measured (5 drones × wide+narrow, 854×480, warp fisheye, idle host): 32.5 →
  38.2 fps per feed; gz time outside the plugin per pass 25.6 → 2.6 ms. The
  single-drone world (one instance) behaves exactly as before.

## Session Addendum (2026-10-04) — GPU fisheye warp in the export plugin + Ogre worker cap shim

### GPU fisheye warp (`ShmCameraExportPlugin`)
- **Why:** with warp-mode feeds the bridge's CPU warp was the per-feed cost
  (28% CPU per bridge); measured 52.8 → 84.4 fps just by pausing the bridges.
- **Template contract:** the drone vis templates' plugin element carries one
  `<warp sensor="…" out_w="…" out_h="…">SPEC</warp>` child per warp-mode
  camera (loop over betaloop's `shm_warp` list); SPEC is EXACTLY the
  bridge's `--warp-fisheye` string (`common.warp_spec_string`, 9/11 fields).
  Configure parses it null-safely; malformed → logged + ignored.
- **Pass** (`GpuWarp`, render thread, right after the camera's render): a
  fullscreen triangle into an RGBA8 output texture/FBO; LUT = RGBA16UI
  texture (ix, iy, wx, wy) built by `BuildWarpLut` — the bridge's
  `warpFisheyeFromRect` math line for line; fragment shader does the bridge's
  8.8 INTEGER bilinear (`(Σ p·w) >> 16` on raw bytes). The final output
  (not the source) goes through the async PBO readback. **Bit-exact** vs the
  bridge: 0 of 409,920 pixels differ.
- **GL traps (each cost a debug cycle):**
  1. The camera RTT is `GL_SRGB8_ALPHA8` (0x8C43): `texelFetch` DECODES sRGB
     even with `GL_SKIP_DECODE_EXT` on the texture or a sampler → dark
     output. Fix: sample a `glTextureView(…, GL_RGBA8, …)` of it (requires
     immutable storage — Ogre's RTT has it), plus our own NEAREST sampler.
  2. Ogre leaves its unpack layout set (ROW_LENGTH/SKIP_*): the LUT upload
     came out garbage. Reset ALL `GL_UNPACK_*` params for the upload and
     restore them after.
  3. Ogre caches GL state — every piece touched (program, FBOs, VAO,
     viewport, active texture, texture+sampler on units 0/1, unpack buffer +
     params, color mask, and the caps blend/depth/scissor/cull/stencil/
     FRAMEBUFFER_SRGB/RASTERIZER_DISCARD/SAMPLE_MASK/ALPHA_TO_COVERAGE/
     DEPTH_CLAMP/COLOR_LOGIC_OP/POLYGON_OFFSET_FILL/CLIP_DISTANCE0-7) is
     saved and restored exactly; pending GL errors are drained first so the
     end-of-pass check only sees ours.
- **CPU fallback (never export an un-warped frame):** the bridge does NOT
  warp these feeds, so if the GPU warp is unavailable/fails, or with
  `SHM_EXPORT_SYNC=1` or `SHM_EXPORT_NO_GPU_WARP=1`, `CpuWarp` runs the same
  LUT + integer bilinear on the render thread (~5.5 ms/frame under load —
  a failure path, not a mode). Async ring slots remember whether they were
  GPU-warped (`ringWarped`). Verified on an isolated bench: GPU, async CPU
  fallback, sync CPU fallback and the bridge's CPU warp are byte-identical.
  The stats line shows `gpu-warp+` vs `CPU-warp(fallback)+`. A source-size
  mismatch DROPS frames with a log line (re-render the models).

### Ogre worker cap (`plugins/OgreWorkerThreads.cc` → `libOgreWorkerThreads.so`)
- LD_PRELOAD shim (CMake target `OgreWorkerThreads`) defining
  `Ogre::PlatformInformation::getNumLogicalCores()`; returns
  `$GZ_OGRE_WORKER_THREADS` when > 0, else the stock logical-core count.
  gz-rendering8 `Ogre2Scene::CreateContext` passes that value as the scene
  manager's worker count and is the only importer (verified with `nm -D`), so
  the interposition only resizes Ogre's culling/update pool. Preloaded into
  the gz process only, by betaloop `gz_spawn_env` (`--ogre-workers`, default
  2). Measured numbers: root CLAUDE.md 2026-10-04.
- Open finding: in a multi-model pass, the cameras of the model rendered
  FIRST cost ~2–3.7 ms more each than the other model's (parallelizable CPU
  work; follows the model, not the position — tested by reversing and
  interleaving the render order). Cause not found (not the Forward+ grid
  cache, heightmaps or the 6-pass GPU flush).
