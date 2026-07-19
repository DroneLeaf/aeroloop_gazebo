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
