# Legacy Controller Reference Extraction

The recovered `WalkingExcursion_v1.py` file is useful evidence, but it should
not be run as the new controller. It configured CAN, enabled motors and zeroed
all motors during startup. The safer path is to extract its non-hardware ideas
into pure, testable modules.

## Extracted Safe Reference Modules

| Module | Purpose | Hardware behavior |
| --- | --- | --- |
| `tocatl_kinematics.legacy_geometry` | Link lengths, body reference, legacy FK/IK branch, and old theta/motor transforms | None |
| `tocatl_gait.legacy_reference` | Wave/ripple/tripod phase offsets and old walking envelope constants | None |

Both modules are reference-only. They do not import CAN, start ROS, open socket
interfaces, enable motors, write zero positions, or send motion commands.

## Useful Recovered Values

| Category | Value |
| --- | --- |
| Upper link | `330.3455 mm` |
| Lower link | `458.0 mm` |
| Body diameter used by old gait model | `600 mm` |
| Gaits | `wave`, `ripple`, `tripod` |
| Nominal gait cycle time | `1.5 s` |
| Slow / fast cycle targets | `4.5 s` / `1.2 s` |
| Default stride length / step height | `175 mm` / `215 mm` |
| Max stride length / step height | `400 mm` / `400 mm` |
| Foot speed cap used by old script | `400 mm/s` |
| Body speed / yaw-rate caps | `140 mm/s`, `35 deg/s` |
| Old command stream rate | `200 Hz` |

These are historical working values. They are not validated safety limits and
should not bypass the new calibration file, command limiter or hardware E-stop.

## Why This Is Better Than Copying the Old Script

- The old source remains preserved as evidence.
- New code can import gait/geometry constants without triggering hardware.
- Unit tests can verify that the extracted values match the recovered source.
- Future ROS 2 nodes can call clean modules instead of a monolithic joystick
  walking script.
- Dangerous operations stay in explicit, reviewed layers: telemetry, arming,
  command limiting and motor I/O.

## Next Step

Use these modules to build a slow, named `park` / `stand` / `small-wave` motion
planner that outputs desired joint positions only. That planner should still
route through:

1. valid calibration,
2. fresh telemetry,
3. `tocatl_control.safety_state`,
4. `tocatl_control.command_limiter`,
5. the physical motor-power safety path.
