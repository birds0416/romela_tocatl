# TOCATL Motor and CAN Map

This map is recovered from the controller deployed on the Raspberry Pi on
2026-03-15. It is an evidence-based starting point, not yet a physical wiring
verification.

| Leg | CAN bus | Hip yaw | Hip pitch | Knee |
|---:|:---:|---:|---:|---:|
| 1 | `can1` | 11 | 12 | 13 |
| 2 | `can0` | 21 | 22 | 23 |
| 3 | `can0` | 31 | 32 | 33 |
| 4 | `can0` | 41 | 42 | 43 |
| 5 | `can1` | 51 | 52 | 53 |
| 6 | `can1` | 61 | 62 | 63 |

- Bus bitrate: 1,000,000 bit/s.
- Motor family: Xiaomi CyberGear, based on the motor label and deployed driver.
- IDs ending in 1, 2, and 3 represent hip yaw, hip pitch, and knee.

Run the offline-only inventory with:

```bash
PYTHONPATH=src/tocatl_diagnostics python3 -m tocatl_diagnostics.motor_map
```

The module has no CAN dependency and cannot send, enable, zero, or command a
motor. Any future live telemetry command must be a separate, explicitly reviewed
tool because even a state request transmits a CAN frame.

