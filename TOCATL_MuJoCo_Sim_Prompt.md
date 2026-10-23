# Build a ROS 2 + MuJoCo walking simulator for the TOCATL hexapod

> Paste this whole document into your coding agent (Fable) from inside the
> `romela-tocatl` repository on your Linux machine. It is a complete build
> brief. Everything the agent needs — geometry, gait math, joint map, control
> scheme, and an accept/iterate loop — is stated here.

---

## 0. Role and goal

You are a robotics software engineer working **inside the existing
`romela-tocatl` ROS 2 workspace**. Your job is to create a **MuJoCo simulation
of the TOCATL hexapod (spider) robot, integrated with ROS 2 Jazzy**, so that the
robot can be **walked around with the keyboard** and the walk can be **screen-
recorded looking firm and neat**.

The real robot is controlled by an Xbox/console controller over CAN. There is no
controller here and this is simulation only, so **keyboard control replaces the
joystick**.

**Success = a stable, upright, non-slipping tripod walk that moves smoothly on
keyboard command and looks clean on camera.** If your first version wobbles,
skates, sinks, or falls over, **keep iterating and tuning until it walks well**
(Section 9 is the debugging ladder).

---

## 1. Hard constraints — read before writing any code

1. **Simulation only. Never touch hardware.** Do **not** import `can`,
   `CyberGearDriver`, `python-cybergear`, or `socketcan`. Do **not** call
   `setup_can*`, `enable()`, `set_zero_position()`, or anything that opens a CAN
   bus. The repo has strict safety rules (see `README.md`,
   `docs/controller-roadmap.md`): legacy scripts enable real motors on startup.
   Your code must be inert with respect to hardware.
2. **Do not modify or run anything under `legacy/`.** Treat it as read-only
   reference. Read `legacy/deployed-2026-03-15/tocatl_testing/WalkingTestv3` and
   `WalkingExcursion_v1.py` only to *understand* the gait; port the math, don't
   execute the files.
3. **Reuse the existing pure-math modules** rather than re-deriving them:
   - `src/tocatl_kinematics/tocatl_kinematics/legacy_geometry.py` — link lengths,
     body reference, FK, IK, joint↔motor transforms. These are hardware-free.
   - `src/tocatl_gait/tocatl_gait/legacy_reference.py` — gait phase offsets, duty
     factors, and the walking envelope constants.
   - `src/tocatl_diagnostics/tocatl_diagnostics/motor_map.py` — the canonical
     18-motor / 6-leg / 2-bus map and joint ordering.
   These modules import cleanly with no CAN dependency. Import them; don't copy.
4. **Add a new package, don't rewrite existing ones.** Put all new sim code in a
   new ament_python package `src/tocatl_sim`. Leave `tocatl_control` and its
   safety state machine, command limiter, calibration, etc. untouched.
5. **Match the user's existing manipulator MuJoCo sim conventions** if one is
   present in a sibling workspace (node naming, launch style, mujoco viewer vs
   `mujoco_ros2_control`). If none is found, follow the architecture in Section 4.

---

## 2. The robot (facts, not guesses)

TOCATL is a **radially symmetric hexapod**: a central body with **6 identical
legs**, each leg having **3 revolute joints** driven by a Xiaomi **CyberGear**
quasi-direct-drive actuator. **18 joints total.**

### 2.1 Leg kinematic chain (base → tip)

Each leg, from the body outward:

1. **hip_yaw** — rotates about the leg-local **Z** (vertical) axis. Swings the
   leg left/right around the mounting point.
2. **hip_pitch** — rotates about the leg-local **Y** axis. Lifts/lowers the thigh.
3. **knee** — rotates about the leg-local **Y** axis. Bends the shin relative to
   the thigh.

### 2.2 Link geometry (from `legacy_geometry.LegGeometry`, millimetres)

| Symbol | Meaning | Value |
|---|---|---|
| `d1` `hip_vertical_offset_mm`   | vertical drop from yaw axis to pitch axis | **114.25** |
| `d2` `hip_horizontal_offset_mm` | radial offset from yaw axis to pitch axis  | **45.5561** |
| `d3` `upper_link_mm`            | thigh (hip_pitch → knee)                   | **330.3455** |
| `d4` `lower_link_mm`            | shin (knee → foot tip)                     | **458.0** |

Max radial reach from the hip ≈ `d2 + d3 + d4 ≈ 834 mm`; practical planted reach
is well inside that.

### 2.3 Body / leg mounting (from `legacy_geometry.BodyReference`)

- Body modelled as a disc of **diameter 600 mm → radius 300 mm**.
- Leg **mount angles**: leg *k* (0-based, k = 0..5) is mounted at
  `30° + k·60°`. So the six hips sit every 60° around the body, with leg 0 (i.e.
  leg 1 in the 1-based motor map) at +30°.
- The hip_yaw axis at each mount points **radially outward** in the body frame.

### 2.4 Forward / inverse kinematics — use the repo's, verify against this

`legacy_geometry.foot_position_from_joint_angles(...)` (FK) and
`inverse_kinematics_from_foot_position(...)` (IK) are the authoritative
leg-local transforms. The equivalent in `WalkingTestv3` is:

```
base = [d2, 0, d1]                               # in leg-local frame
knee = base + Rz(θ1) · Ry(θ2)        · [d3,0,0]
foot = knee + Rz(θ1) · Ry(θ2+θ3)     · [d4,0,0]
```

IK: `θ1 = atan2(y, x)`; solve the planar 2-link (`d3`,`d4`) problem in the
radial-vertical plane for `θ2`, `θ3` using the law of cosines (positive knee
branch), exactly as `inverse_kinematics_from_foot_position` does. **Unit-test
FK(IK(p)) ≈ p** for a grid of reachable foot points before trusting the gait.

### 2.5 Joint / motor map (from `motor_map.py` and `docs/motor-map.md`)

18 motors, IDs `leg*10 + joint` where joint 1=hip_yaw, 2=hip_pitch, 3=knee:

```
Leg 1: 11 hip_yaw  12 hip_pitch  13 knee   (real bus can1)
Leg 2: 21          22            23        (real bus can0)
Leg 3: 31          32            33        (real bus can0)
Leg 4: 41          42            43        (real bus can0)
Leg 5: 51          52            53        (real bus can1)
Leg 6: 61          62            63        (real bus can1)
```

**Keep this exact ordering** for the 18-element joint vectors so the sim maps
1:1 onto the real robot later. (Buses are irrelevant in sim; keep them only in
comments/metadata.)

### 2.6 Recovered joint windows (leg/theta frame, radians)

From `legacy_geometry.LegacyMotorMapping` — use as starting joint ranges, widen
only if the workspace needs it:

- hip_yaw: about **−1.0 … +1.0**
- hip_pitch: about **0.0 … 4.9** (motor window; map through the θ↔motor
  transform, don't apply blindly to θ)
- knee (relative): about **−5.0 … 0.0** (motor window; likewise map through)

Use `legacy_geometry.theta_to_legacy_motor` / `legacy_motor_to_theta` /
`apply_legacy_motor_windows` to convert. For the MJCF, set generous mechanical
`range` on each hinge and let the gait planner stay inside the reachable
workspace.

### 2.7 Actuator model — CyberGear MIT mode (match this exactly)

The real streamer sends MIT-style commands `(position, velocity, kp, kd, torque)`
per motor and the motor applies:

```
τ = kp·(pos_des − q) + kd·(vel_des − q̇) + τ_ff
```

**Reproduce this in sim** with **torque (`motor`) actuators** and compute τ in
software from the incoming command. This gives you the same per-joint,
per-phase gain scheduling the real controller uses. CyberGear peak torque
≈ **12 N·m** → set `forcerange="-12 12"` (ctrlrange to match). The real hip-pitch
support-torque cap observed in the legacy code is **9.25 N·m**, a useful sanity
bound.

---

## 3. Gait (this is what makes it walk — port it faithfully)

Constants live in `tocatl_gait/legacy_reference.py`; the full trajectory lives in
`legacy/.../WalkingTestv3`. Reproduce the following.

### 3.1 Phase engine

- One global phase `u ∈ [0,1)` advances at `du = dt / cycle_time`.
- Per leg *i*: `u_i = (u + phase_offset[i]) mod 1`.
  - If `u_i < duty_factor` → **stance**, progress `s = u_i / duty`.
  - Else → **swing**, progress `s = (u_i − duty) / (1 − duty)`.
- Helper `legacy_reference.phase_for_leg(gait, leg_index, u)` already returns
  `(u_i, is_stance, s)`. Use it.

### 3.2 Gaits (from `legacy_reference`)

| Gait | duty | phase offsets (legs 0..5) | character |
|---|---|---|---|
| **tripod** | 0.5   | `0, .5, 0, .5, 0, .5`            | 3 legs swing at once — **default, most stable/fast** |
| **ripple** | 2/3   | `0, 1/3, 2/3, 0, 1/3, 2/3`      | 2 legs swing — smoother |
| **wave**   | 5/6   | `5/6, 1/3, 2/3, 1/6, 1/2, 0`   | 1 leg swings — slowest, most stable |

Nominal `cycle_time = 1.5 s`; fast `1.2 s`, slow `4.5 s`.

### 3.3 Foot trajectory (per leg, in the leg's neutral world/body frame)

For each leg define a **neutral foot point**: radially out from its hip at
`R_foot` (start ≈ **400–450 mm** from body centre), on the ground plane, with the
body at height `H_body` (start ≈ **280–320 mm**). Then:

- **Travel direction** `\hat{t}` and **stride length** `L` come from the commanded
  body velocity `(vx, vy)` (scaled by speed, clamped to `stride_max`). Yaw command
  `ω` adds a tangential per-leg component (`YAW_STRIDE_SCALE ≈ 0.2`).
- **Stance** (foot planted, pushes body forward): move the foot from
  `+L/2·\hat{t}` to `−L/2·\hat{t}` **linearly in `s`**, held on the ground
  (`z = 0`). In the world the foot stays put while the body advances → no slip.
- **Swing** (foot in air, returns to front): move from `−L/2·\hat{t}` back to
  `+L/2·\hat{t}`, lifting with `z = step_height · sin(π·s)` (a clean arch; a
  cycloid/Bézier is fine too). Start `step_height ≈ 180–220 mm`.
- Convert each foot target from body frame → that leg's local frame (subtract hip
  mount position, rotate by `−(mount_angle + body_yaw)`), run **IK**, get
  `(θ1, θ2, θ3)`, then map to the 18-vector.

### 3.4 Envelope defaults & limits (from `LegacyWalkingEnvelope`)

```
default stride  175 mm   (min 20,  max 400)
default step_h  215 mm   (min 0,   max 400)
foot speed max  400 mm/s
body speed max  140 mm/s
body yaw rate   35 °/s
command stream  ~200 Hz  (run the gait/command loop at 200 Hz)
```

### 3.5 Phase-scheduled gains (the key to a "firm" look)

Stance legs must be **stiff** (plant hard, carry weight); swing legs **soft**
(compliant, don't fight the air). From `WalkingTestv3`:

```
stance  Kp ≈ 100   |  swing Kp ≈ 10   |  Kd ≈ 3   (per joint group)
```

Ramp Kp up quickly entering stance, down slowly entering swing (the legacy
`KP_SCHEDULE_RATE_UP/DOWN` behaviour). Pass `(pos, vel, kp, kd)` per joint into
the sim node so it can apply the MIT-mode torque of Section 2.7.

---

## 4. Architecture (ROS 2 Jazzy, integrated — the chosen design)

Create package **`src/tocatl_sim`** (ament_python) with these nodes:

1. **`mujoco_sim_node`** — the physics + rendering server.
   - Loads the MJCF (`tocatl_sim/model/tocatl.xml`).
   - Subscribes **`/joint_commands`** (recommend `sensor_msgs/JointState` with
     `name`, `position`, `velocity`, plus a parallel `Float64MultiArray` for the
     per-joint `kp`/`kd`, or a small **custom msg** `JointCommand[]` carrying
     `pos, vel, kp, kd, tau`). Applies MIT-mode torque, steps MuJoCo at a fixed
     timestep (e.g. `dt = 0.002 s`, i.e. 500 Hz physics).
   - Publishes **`/joint_states`** (`sensor_msgs/JointState`, all 18 joints in the
     Section 2.5 order) and the free-body pose (**`/odom`** or a TF
     `world→base_link`).
   - Renders: `mujoco.viewer.launch_passive` for the live window; an optional
     offscreen `mujoco.Renderer` path for recording (Section 8).
2. **`gait_node`** — the locomotion brain.
   - Subscribes **`/cmd_vel`** (`geometry_msgs/Twist`: `linear.x=vx`,
     `linear.y=vy`, `angular.z=yaw_rate`) and **`/gait_params`** (gait name,
     stride, step height, body height — a custom msg or `std_msgs` combo).
   - Runs the phase engine + foot trajectory + IK from Section 3 at 200 Hz,
     imports `tocatl_kinematics.legacy_geometry` and
     `tocatl_gait.legacy_reference`, publishes **`/joint_commands`** with
     phase-scheduled gains.
   - Holds a **stand pose** when velocity is ~0 (feet planted at neutral, stance
     gains) so the robot stays rock-steady between moves.
3. **`teleop_keyboard`** — the driver.
   - Reads the keyboard (termios raw stdin, or `pynput`) and publishes `/cmd_vel`
     + `/gait_params`. Non-blocking; prints a live help/status line.

Provide a **launch file** `tocatl_sim/launch/sim.launch.py` that starts all three
plus RViz-optional, and a **params yaml** `tocatl_sim/config/sim.yaml` with all
tunables (masses, gains, envelope, timestep, camera).

> Rationale for keeping the gait outside the physics node: it mirrors the real
> robot's node split (`gait generator → command sender → motors`) from
> `docs/software-backlog.md`, so the same `gait_node` can later target real CAN
> via a different backend without touching locomotion logic.

### 4.1 Keyboard map (replaces the Xbox controller)

Original pad → keyboard equivalents:

| Key | Action | Original pad |
|---|---|---|
| `W` / `S` | walk forward / backward (vx) | Left stick Y |
| `A` / `D` | strafe left / right (vy) | Left stick X |
| `Q` / `E` | turn (yaw) left / right | Right stick X |
| `1` / `2` / `3` | select tripod / ripple / wave | (gait select) |
| `[` / `]` | stride length − / + | DPAD X |
| `-` / `=` | step height − / + | DPAD Y |
| `R` / `F` | body height up / down | A / B |
| `T` / `G` | leg radius out / in | Y / X |
| `Space` | stop → graceful stand | LB (graceful return) |
| `Z` | park (lower + settle) | RB |
| `X` or `Esc` | quit sim cleanly | Back (E-stop) |

Speeds ramp toward the commanded value (slew, don't step) so motion is smooth on
camera. Hold-to-move or press-to-toggle is your call — **hold-to-move reads
better in a recording**; if the keyboard backend can't do key-repeat cleanly,
use press-to-set-velocity with `Space` to stop.

---

## 5. The MuJoCo model (`tocatl.xml`) — build parametrically

Generate the MJCF **from the Section 2 numbers** (parametric capsules — no CAD
meshes needed). Prefer generating the XML from a small Python builder that loops
over the 6 legs using the mount angles, so geometry stays consistent.

- **Units:** MuJoCo is SI — convert all mm to **metres**.
- **World:** ground `plane` with friction, `gravity = 0 0 -9.81`.
- **Base:** one body `base_link` with a **`freejoint`**; visual a hexagonal/disc
  plate radius 0.30 m, small height (~0.10 m). Spawn at height `H_body` so legs
  start on the ground.
- **Per leg (×6):** at mount angle `30°+k·60°`, radius 0.30 m:
  - `coxa` body, hinge **hip_yaw** about local Z, small capsule for the `d2`
    offset.
  - `femur` body, hinge **hip_pitch** about local Y, capsule length `d3=0.330 m`.
  - `tibia` body, hinge **knee** about local Y, capsule length `d4=0.458 m`.
  - **foot** geom: small sphere at the tip with **high friction**
    (`condim=6`, `friction="1.2 0.02 0.001"`) and a little `solimp/solref`
    softness so contacts don't jitter.
- **Actuators:** one `motor` (torque) actuator per hinge, `forcerange="-12 12"`.
  (Alternative: `position` actuators with `kp`, but you lose per-phase gain
  scheduling — prefer `motor`.)
- **Mass / inertia (estimates — tune in Section 9):** total robot ≈ **12 kg**.
  Suggest `base_link ≈ 6–7 kg`; per leg `coxa ≈ 0.5`, `femur ≈ 0.6`, `tibia ≈
  0.4 kg` (CyberGear ≈ 0.3–0.5 kg each dominate link mass — place coxa/femur mass
  near the joints). Let MuJoCo compute inertia from geoms with a sane `density`,
  or set explicit `inertial`. Keep the **CoM low and central** for stability.
- **Sensors (optional but useful):** `framepos`/`framequat` on `base_link`,
  `jointpos`/`jointvel` on each hinge, `touch` on each foot — handy for the
  headless acceptance test.

---

## 6. Reuse vs. build — checklist

- [ ] Import geometry/IK from `tocatl_kinematics.legacy_geometry`.
- [ ] Import gaits/envelope from `tocatl_gait.legacy_reference`.
- [ ] Import the joint order from `tocatl_diagnostics.motor_map`.
- [ ] New package `tocatl_sim`: `mujoco_sim_node`, `gait_node`, `teleop_keyboard`,
      `model/tocatl.xml` (+ builder), `launch/sim.launch.py`, `config/sim.yaml`,
      `test/`, `README.md`.
- [ ] `package.xml` deps: `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`,
      `tf2_ros`, and the three tocatl packages. `mujoco` via pip.
- [ ] Nothing imports CAN / CyberGear driver / socketcan.

---

## 7. Environment & build (Ubuntu 24.04 + ROS 2 Jazzy)

```bash
# from repo root (the ROS 2 workspace)
python3 -m venv .venv && source .venv/bin/activate   # matches README setup
pip install --upgrade pip
pip install mujoco numpy scipy
# ROS 2 Jazzy assumed already installed (ros-jazzy-desktop)
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tocatl_sim tocatl_kinematics tocatl_gait tocatl_diagnostics
source install/setup.bash
ros2 launch tocatl_sim sim.launch.py
```

If the venv/ROS interplay is awkward, use `--system-site-packages` on the venv or
install `mujoco` into the environment ROS uses. Confirm `python3 -c "import
mujoco, rclpy"` works before building nodes.

---

## 8. Recording the walk

Provide a clean way to capture video (the user will record the walk):

- **Live viewer + screen capture:** launch the passive viewer, set a fixed,
  slightly-elevated 3/4 camera in the MJCF (`<camera>`), and let the user OBS /
  screen-record. Add a `--camera` launch arg to select it.
- **Built-in offscreen recorder (preferred, reproducible):** a `record` mode in
  `mujoco_sim_node` (or a small `record_node`) that uses `mujoco.Renderer` to
  render frames at a fixed FPS while the sim runs a scripted or live-driven walk,
  and writes `walk.mp4` via `imageio[ffmpeg]` (`pip install imageio imageio-ffmpeg`).
  Expose `--seconds`, `--fps`, `--camera`, `--out`.
- Frame the shot on a smooth **tripod walk forward** for a few seconds, then a
  gentle turn. Keep the camera static; a steady body and non-slipping feet are
  what make it look professional.

---

## 9. Iterate until it walks well (do not stop at "it compiles")

Bring it up in this order, fixing at each rung before advancing. Add a **headless
acceptance test** (`test/test_walk.py`) that runs the sim without a window and
asserts the metrics below, so you can iterate automatically before ever opening
the viewer.

1. **Static stand.** Command zero velocity; robot holds the stand pose for 10 s.
   - *Pass:* base height stable within ±10 mm, roll & pitch < 2°, no foot slip.
   - *If it sags/collapses:* raise stance Kp, lower total mass or CoM, check IK
     sign so knees bend the correct way, verify feet actually reach the ground at
     `H_body`.
2. **March in place.** Tripod, stride 0, step height ~150 mm.
   - *Pass:* alternating tripods lift/plant cleanly, body barely moves, no jitter.
   - *If it buzzes:* soften contact (`solref`), lower physics timestep to 0.001,
     reduce swing Kp, add joint `damping`.
3. **Slow tripod forward.** `vx` small, cycle 1.5 s.
   - *Pass:* body advances smoothly; **stance feet stay fixed in world (no
     skating)**; forward distance ≈ commanded speed × time (±20%).
   - *If feet skate:* increase foot friction, raise stance Kp, verify stance
     trajectory moves the foot **backward in world at exactly body speed**, make
     sure ≥3 feet are always in stance (tripod) and truly loaded.
4. **Faster + turning.** Increase `vx`, add yaw; test ripple and wave too.
   - *Pass:* stays upright while turning, gait switches without a stumble.
5. **Camera pass.** Run the recorder; watch for wobble, toe-drag, or hip pop and
   tune it out. **This is the bar: it must look firm and neat.**

**Acceptance metrics for `test/test_walk.py` (tripod, 5 s, nominal speed):**
- forward displacement ≥ 0.15 m and within ±25% of commanded,
- mean |roll|, |pitch| < 3°, base-height std-dev < 15 mm,
- no base contact with ground, no joint NaNs,
- per-foot horizontal slip during its stance phase < 15 mm.

Tune masses, gains, friction, timestep, stride, step height, and cycle time until
these pass **and** the viewer looks clean. Log what you changed and why.

---

## 10. Deliverables

1. `src/tocatl_sim/` package building cleanly with `colcon`.
2. `ros2 launch tocatl_sim sim.launch.py` → MuJoCo window with the hexapod
   standing, drivable by keyboard per Section 4.1.
3. Stable **tripod walk** forward/back/strafe/turn; gait switch 1/2/3.
4. `test/test_walk.py` headless acceptance test passing the Section 9 metrics.
5. A `record` path that outputs `walk.mp4`.
6. `tocatl_sim/README.md`: how to build, launch, drive, record, and the tuned
   parameter values you settled on (with a short note on what you changed to get a
   firm walk).

Keep it clean, commented, and faithful to the recovered geometry and gait. When
the walk looks solid on camera, you're done.
