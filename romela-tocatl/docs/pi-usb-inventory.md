# Pi and USB Inventory — Motor Rail Off

This is Stage 1 of TOCATL bring-up. It identifies the Pi, USB devices, network
links, and old controller-service state without opening CAN or commanding a
motor.

## Physical gate

Before connecting USB-to-CAN adapters or running the inventory:

- power the Pi from its independent USB-C supply
- disconnect the main robot battery so the motor rail is off
- confirm `tocatl.service` remains disabled and inactive
- keep the Xbox controller off
- support the robot and keep people clear even though motion is not expected
- label each USB-to-CAN adapter and record which physical cable/side it serves

If the main battery or motor rail cannot be positively identified as off, stop.

## Run over SSH

From the repository on the Pi:

```bash
cd ~/Desktop/dev/romela_tocatl
git fetch origin
git switch main
git pull --ff-only origin main
PYTHONPATH=src/tocatl_diagnostics \
  python3 -m tocatl_diagnostics.pi_inventory \
  | tee ~/tocatl-pi-usb-inventory.json
```

Do not run `ip link set`, `cansend`, a CyberGear driver, or any recovered
controller script during this stage.

Copy the result back to the Mac:

```bash
scp tocatl@192.168.2.2:~/tocatl-pi-usb-inventory.json .
```

Before saving the result in Git or Drive, inspect USB serial fields and remove
anything the lab considers sensitive. The inventory intentionally omits IP
addresses and Wi-Fi configuration.

## Pass criteria

- Pi model and kernel are recorded.
- Both USB-to-CAN adapters can be distinguished by vendor/product, serial, USB
  path, or a durable physical label.
- Expected CAN/network interface names are visible or their absence is noted.
- `tocatl.service` reports `disabled` and `inactive`.
- No motor power was applied and no CAN frame was transmitted.
