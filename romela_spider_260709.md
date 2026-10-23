# RoMeLa TOCATL Spider — MuJoCo Sim Session Log (2026-07-09)

Session goal: per `TOCATL_MuJoCo_Sim_Prompt.md`, build a ROS 2 Jazzy + MuJoCo
walking simulator for the TOCATL hexapod inside the `romela-tocatl` workspace,
drivable by keyboard — then (follow-up request) make it look like the real
robot using the SolidWorks CAD in `TOCATL_CAD/`.

---

## Phase 1 — The walking simulator

New ament_python package **`src/tocatl_sim`** (simulation only — no CAN, no
motor drivers, `legacy/` untouched):

- **`mujoco_sim_node`** — 500 Hz physics. Applies the CyberGear MIT law
  `τ = kp·(q_des−q) + kd·(q̇_des−q̇) + τ_ff`, clipped ±12 N·m, from the
  200 Hz `/joint_commands` (JointState) + `/joint_gains` (kp×18, kd×18)
  stream. Publishes `/joint_states`, `/odom`, TF `world→base_link`.
  Passive viewer window unless `headless:=true`.
- **`gait_node`** — faithful port of the recovered `WalkingTestv3` gait:
  world-frame foot anchors, sinusoidal swing arcs, tripod/ripple/wave from
  `legacy_reference`, cycle time scheduled from the 400 mm/s foot-speed cap,
  leg-by-leg graceful return (order 1-3-5-2-4-6), park/unpark. Reuses
  `legacy_geometry` FK/IK/motor-windows and the `motor_map` joint order
  (legs 1..6 × hip_yaw / hip_pitch / knee = motor IDs 11..63).
- **`teleop_keyboard`** — hold-to-move via pynput (global key capture),
  raw-terminal fallback. Full key map from the brief (Section 4.1).
- **`model_builder` → `model/tocatl.xml`** — parametric MJCF from
  `legacy_geometry`. Kinematic subtlety discovered: the legacy controller
  rotates hip-yaw about the point `[d2, 0, d1]` (the pitch-axis point), not
  the mount. The MJCF reproduces this exactly — verified < 1 µm against the
  legacy FK over random joint configurations, so IK targets track with zero
  kinematic error.
- **`test/test_walk.py`** — headless acceptance suite (7 tests): FK(IK)
  round-trip grid, model-vs-legacy-FK identity, 10 s static stand, 5 s
  nominal tripod walk (displacement within ±25 % of commanded, mean
  roll/pitch < 3°, height σ < 15 mm, loaded-foot slip < 15 mm, no base
  contact, no NaNs), ripple/wave stability, turning + mid-walk gait switch.
- **`record_walk`** — reproducible offscreen recorder → `walk.mp4`
  (EGL + OpenCV), plus `launch/sim.launch.py` and `config/sim.yaml`.

### Tuning log (what made the walk firm)
| Change | Value | Why |
|---|---|---|
| stance/swing Kp, Kd | 100 / 10, 3.0 | legacy values, kept |
| `kp_rate_up` | **25** (legacy 1.0) | legacy's slow stance ramp planted feet soft → touchdown drag |
| foot radius | 12 mm | less sphere-rolling artifact |
| masses | base 6.0 kg, ~13.6 kg total | CoM low/central; knee torque ≈6 N·m in tripod |
| integrator | 2 ms, implicitfast, elliptic cone | stable with Kp 100 |

Measured (5 s full-speed tripod): 0.36 m travelled (−2 % vs commanded), max
roll/pitch < 0.8°, height σ 6 mm, loaded-foot slip 2–4 mm. Important
measurement lesson: slip must be judged during **loaded contact** (touch
force > 5 N) — measuring across the phase-mask window shows a ~26 mm
artifact from swing-transition frames.

---

## Phase 2 — Real CAD visuals (after seeing the robot photo)

The user provided a photo of the physical TOCATL: black 6-petal dome with
arched leg openings and white trim, A-frame femurs with a parallel knee bar,
CyberGear drums at hips and knees, long tapered shins, wide low stance with
knees above the dome. That photo became ground truth.

### Getting the CAD out of SolidWorks format
- `.SLDPRT/.SLDASM` (modern, non-OLE) is unreadable by every offline tool;
  ImageToStl/fabconvert admit they can't; AnyConv's backend errors;
  Aspose doesn't accept it; CAD Exchanger is behind an account.
- **convert3d.org works — and converts in-browser via WASM**, so the CAD
  never left this machine. Driven with Playwright.
- Automation lessons: the app lives at `/convert`; never click the sidebar
  "Convert" nav while waiting (it resets the app — masqueraded as a quota
  for hours); the Download button is flaky under automation — **fetch the
  page's `blob:` URL directly** instead (always works).
- Multi-file conversion (needed for assemblies → exact placement) is a paid
  feature; declined, so parts were placed by hand against the photo.
- All 17 structural parts converted → `TOCATL_CAD/converted_stl/`.
  Conversion/prep scripts preserved at `TOCATL_CAD/conversion_scripts/`
  (`batch8.py` converter, `prep_meshes.py` mesh prep + artifact filtering,
  `compose2.py` layout-iteration renderer).

### Partial-tessellation repairs
- Lofted shell parts export empty/fragmentary (all Body Shell petals, Femur
  Knee, one bearing attachment) → the **dome is rebuilt parametrically**
  (`dome_builder.py`: creased petals, arched openings, light trim band).
- Several parts contain stacked duplicate discs/cones that decimate into
  spikes → fixed by trimesh connected-component filtering.
- Missing bearing attachment mirrored from its intact twin; CyberGear motors
  are stylized Ø86×36.5 cylinders (motor CAD folder only had shaft parts).

### Placement & stance
- `cad_layout.py` maps each part onto the kinematic skeleton. MuJoCo euler =
  scipy intrinsic `'XYZ'` (verified numerically; beware tiny test models
  default to **degrees** without `angle="radian"`).
- Hip turret transform derived by cylinder-fitting its motor bore (drum axis
  = part x; centre at y 59.5 / z 52.7 mm).
- Stance rematched to the photo: **body 220 mm, stance_radius 250 mm** →
  0.59 m foot ring, thigh up ~26°, knees above the dome.
- **`impratio=10`** added — kills the slow outward foot creep the sprawled
  stance caused (10 mm/10 s → 2 mm/10 s).
- Mesh geoms are massless/contactless; hidden primitives keep the mass, so
  dynamics are identical with `use_meshes:=false`. All 7 tests still pass.

---

## Deliverables
- `~/Desktop/romela_tocatl/walk.mp4` — 18 s camera pass with the CAD body
  (stand → tripod walk → gentle turn → graceful stand), plus `sim_vs_real.png`.
- Commit **`f3686d9`** on branch **`codex/tocatl-mujoco-sim`** (39 files).
  Repo-local git identity set (hojung / birds8277@gmail.com). **Not pushed
  yet** — repo workflow wants push + draft PR + software-safety CI.

## How to run
```bash
cd ~/Desktop/romela_tocatl/romela-tocatl
export PATH=/usr/bin:$PATH        # conda base hides rclpy
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch tocatl_sim sim.launch.py
```
Focus the MuJoCo window and hold **W/S** (fwd/back), **A/D** (strafe),
**Q/E** (turn); **1/2/3** gait; **[ ] - =** stride/step height; **R/F** body
height; **T/G** leg radius; **Space** graceful stand; **Z** park; **X/Esc**
quit teleop. Record: `MUJOCO_GL=egl ros2 run tocatl_sim record_walk`.
Tests: `python3 -m pytest src/tocatl_sim/test/test_walk.py`.

## Open items
- Push branch + open draft PR (awaiting go-ahead).
- Knee bar is fixed to the femur at nominal angle (not an articulating 4-bar).
- No RoMeLa logo on the dome.
- Exact-CAD petals possible if Body Shell STLs are exported from SolidWorks
  (drop into `TOCATL_CAD/exports/`).
