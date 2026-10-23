# tocatl_sim — MuJoCo walking simulator for TOCATL

ROS 2 Jazzy + MuJoCo simulation of the TOCATL hexapod, drivable with the
keyboard. **Simulation only:** nothing in this package imports CAN, the
CyberGear driver, or socketcan, and nothing under `legacy/` is executed. The
gait is a faithful port of the recovered `WalkingTestv3` controller; geometry
and IK are imported from `tocatl_kinematics.legacy_geometry`, gait timing from
`tocatl_gait.legacy_reference`, and the 18-joint ordering follows
`tocatl_diagnostics.motor_map` (leg 1..6 × hip_yaw / hip_pitch / knee, i.e.
motor IDs 11,12,13,21,…,63).

## Build

```bash
cd <repo root>
source /opt/ros/jazzy/setup.bash
# deps into whatever python ROS uses (system python or a --system-site-packages venv):
pip install mujoco numpy opencv-python pynput
colcon build --packages-select tocatl_kinematics tocatl_gait tocatl_diagnostics tocatl_sim
source install/setup.bash
```

> On this machine conda's base env shadows system python — build/run with
> `export PATH=/usr/bin:$PATH` (or `conda deactivate`) so `python3` is the
> interpreter that has `rclpy`.

## Run

```bash
ros2 launch tocatl_sim sim.launch.py                 # viewer + gait + teleop
ros2 launch tocatl_sim sim.launch.py headless:=true teleop:=false
ros2 run tocatl_sim teleop_keyboard                  # teleop alone, any terminal
```

The robot spawns standing. With `pynput` installed the movement keys are true
hold-to-move and are captured globally on X11, so you can drive while the
MuJoCo window has focus. Without pynput, teleop falls back to a raw-terminal
press-to-set mode (keys step the velocity, Space stops).

| Key | Action | | Key | Action |
|---|---|---|---|---|
| `W`/`S` | forward / backward | | `1`/`2`/`3` | tripod / ripple / wave |
| `A`/`D` | strafe left / right | | `[`/`]` | stride − / + |
| `Q`/`E` | turn left / right | | `-`/`=` | step height − / + |
| `R`/`F` | body up / down | | `T`/`G` | leg radius out / in |
| `Space` | stop → graceful stand | | `Z` | park / unpark |
| `X`/`Esc` | quit teleop | | | |

## Topics

- `/cmd_vel` (`geometry_msgs/Twist`) — physical units; clamped to the legacy
  envelope (0.140 m/s translation, 35 °/s yaw).
- `/gait_cmd` (`std_msgs/String`, JSON) — `{"action": "set_gait"|"adjust"|"stop"|"park"|"unpark", ...}`.
- `/joint_commands` (`sensor_msgs/JointState`) — 200 Hz MIT-mode stream:
  `position`, `velocity`, `effort` = feedforward torque.
- `/joint_gains` (`std_msgs/Float64MultiArray`) — `[kp×18, kd×18]`.
- `/joint_states`, `/odom`, TF `world→base_link` — from the physics node.

The physics node applies the CyberGear MIT law per 2 ms step,
`τ = kp(q_des−q) + kd(q̇_des−q̇) + τ_ff`, clipped to ±12 N·m — the same
per-joint, per-phase gain scheduling path the real streamer uses, so
`gait_node` could later target real CAN unchanged.

## Record

```bash
MUJOCO_GL=egl ros2 run tocatl_sim record_walk -- --out walk.mp4 --seconds 18 --fps 60
```

Scripted camera pass (stand → tripod walk → gentle turn → graceful stand)
rendered offscreen at 1280×720 from the `track34` follow camera (`--camera
front|fixed34`, `--gait ripple|wave` also available). Live-viewer screen
capture works too; the same cameras exist in the MJCF.

## Test

```bash
python3 -m pytest src/tocatl_sim/test/test_walk.py -v
```

Headless acceptance suite (Section 9 of the sim brief): FK(IK) round-trip,
MJCF-vs-legacy-FK identity, 10 s static stand, 5 s nominal tripod walk
(displacement within ±25 % of commanded, mean |roll|,|pitch| < 3°, height
σ < 15 mm, loaded-foot slip < 15 mm, no base contact, no NaNs), ripple/wave
stability, turning and mid-walk gait switching.

## CAD visuals

The sim renders the real TOCATL CAD: 17 structural parts from `TOCATL_CAD/`
(SolidWorks) were converted to STL via convert3d.org's in-browser WASM
converter (files never left the machine; scripts in `/tmp/cad_convert`),
decimated, and placed on the kinematic skeleton per `cad_layout.py`. Mesh
geoms are visual-only (no contact, no mass) — the hidden primitive geoms keep
carrying mass/inertia, so dynamics are identical with `use_meshes:=false`.
Converted meshes live in `meshes/` (installed to the package share).

Known substitutions, because the WASM tessellator cannot handle some lofted
surface parts: the 6-petal printed body shell is rebuilt parametrically
(`dome_builder.py`, creased petals + arched leg openings + light trim, matched
to the real robot); the CyberGear motors (86 x 36.5 mm) are stylized cylinders;
`Femur Knee` and one bearing-attachment side were skipped/mirrored. The stand
posture was matched to the real robot (body 220 mm, feet at a 0.59 m ring,
knees above the dome), and `impratio=10` was added to kill slow stance creep
from the sprawled legs. All acceptance tests pass at this stance.

## Model notes

`model/tocatl.xml` is generated — do not hand-edit; run
`python3 -m tocatl_sim.model_builder -o src/tocatl_sim/model/tocatl.xml` after
changing `model_builder.py`. Geometry comes from `legacy_geometry` (d1 114.25,
d2 45.56, d3 330.35, d4 458.0 mm; body Ø600 mm; mounts at 30°+k·60°). The
hip_yaw hinge is anchored at the pitch-axis point `[d2, 0, d1]`, matching the
legacy FK convention exactly (the legacy controller rotates yaw about that
point, not the mount), so IK targets track with zero kinematic mismatch —
verified to <1 µm in the tests. Only the feet and base plate collide; leg
capsules are visual, which avoids leg-leg snags without affecting the walk.

## Tuned parameters (what changed to get a firm walk)

| Parameter | Value | Note |
|---|---|---|
| stance/swing Kp, Kd | 100 / 10, 3.0 | legacy WalkingTestv3 values, kept |
| `kp_rate_up` | **25** (legacy 1.0) | slow stance ramp planted soft and let feet drag ~mm at touchdown; fast ramp plants hard = firm look |
| `kp_rate_down` | 6.0 | legacy value |
| foot radius | **12 mm** (was 20) | smaller sphere-rolling artifact during shin rotation |
| foot friction | 1.2 / 0.02 / 0.001, `condim=6` | prompt values, sufficient — measured loaded-contact slip 2–4 mm |
| masses | base 6.0, per leg 1.26 kg (≈13.6 total) | CoM low and central; static knee torque ≈6 N·m in tripod, inside the 12 N·m limit |
| timestep / integrator | 2 ms, `implicitfast`, elliptic cone | stable with Kp 100 + armature 0.01 |
| stance geometry | body 300 mm, foot ring Ø ≈1.09 m | wider than legacy radius_offset default; better support polygon and knee lever arm |
| stride / step height | 175 / 215 mm | legacy envelope defaults; at full stride+height the 400 mm/s foot-speed cap stretches the cycle to ≈2.3 s and body speed to ≈72 mm/s, exactly as the legacy scheduler did |

Result on the acceptance run: 5 s tripod walk at full command = 0.36 m
travelled (−2 % vs commanded), max |roll| 0.6°, max |pitch| 0.8°, height
σ 6 mm, loaded-foot slip max 4.3 mm.

Sim-vs-real deltas to remember: `park` lowers and settles but does not zero
motors (no calibration writes in sim), and Space's "graceful stand" reuses the
legacy leg-by-leg return sequence (order 1-3-5-2-4-6).
