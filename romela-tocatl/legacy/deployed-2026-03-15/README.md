# Recovered TOCATL Deployment — 2026-03-15

This directory preserves controller artifacts recovered from the TOCATL
Raspberry Pi on 2026-07-01. The dated deployment label comes from the deployed
walking-controller file timestamp and the most recent service failure log.

## Source

- Pi: Raspberry Pi 5 Model B Rev 1.0, 8 GB.
- Hostname: `tocatl`.
- Operating system observed: Ubuntu 24.04.3 LTS ARM64.
- Pi repository commit: `98a4f9ad002d2770dedd81f6fc1ac4f595078b04`.
- Original controller location:
  `/home/tocatl/Desktop/dev/romela_tocatl/build/tocatl_testing/`.
- Original user service:
  `/home/tocatl/.config/systemd/user/tocatl.service`.

The `build/` directory was ignored by Git. These files were therefore absent
from the public repository even though the service executed one of them.

## Preserved Content

- `tocatl_testing/`: ignored test and controller scripts copied from the Pi
  snapshot without intentional source edits.
- `systemd/tocatl.service`: service definition that launched
  `WalkingExcursion_v1.py` with `Restart=always`.
- `SHA256SUMS`: integrity hashes for recovered artifacts.

## Safety Warning

Do not run these scripts on a robot or CAN-connected computer. Static review of
`WalkingExcursion_v1.py` found that it can configure both CAN buses, enable all
18 motors, and write zero positions without waiting for controller readiness or
explicit operator arming. The service's restart policy can also relaunch the
controller after a process-exiting stop.

The Pi service was backed up, disabled, and verified inactive on 2026-07-01.
This recovery branch is preservation evidence, not an approved deployment.

## Development Rule

Keep this directory unchanged. Build replacement behavior in separate branches
and tracked source packages. Any intentional normalization of recovered code
should occur in a new directory with a documented comparison to these hashes.

## Excluded

- Pi password and account secrets.
- SSH private keys and Wi-Fi profiles.
- Python virtual environment.
- Generated ROS install/log trees.
- Unrelated home-directory data.
