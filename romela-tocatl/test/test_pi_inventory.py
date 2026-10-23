from pathlib import Path
import sys
import tempfile
import unittest


PACKAGE_ROOT = Path(__file__).parents[1] / "src" / "tocatl_diagnostics"
sys.path.insert(0, str(PACKAGE_ROOT))

from tocatl_diagnostics.pi_inventory import COMMANDS, collect, usb_devices  # noqa: E402


class PiInventoryTests(unittest.TestCase):
    def test_command_allowlist_contains_only_observation_commands(self):
        rendered = [" ".join(command) for command in COMMANDS]
        self.assertEqual(rendered, [
            "lsusb",
            "ip -details link show",
            "systemctl --user is-enabled tocatl.service",
            "systemctl --user is-active tocatl.service",
        ])
        forbidden = (" up", " down", " set ", "sudo", "can-send", "cansend")
        self.assertFalse(any(token in command for command in rendered for token in forbidden))

    def test_collection_uses_only_the_allowlist(self):
        observed = []

        def fake_runner(command):
            observed.append(command)
            return {"command": list(command), "returncode": 0}

        result = collect(runner=fake_runner)
        self.assertEqual(observed, list(COMMANDS))
        self.assertIn("motor rail off", result["safety_notice"])

    def test_usb_sysfs_parser_collects_identity_fields(self):
        with tempfile.TemporaryDirectory() as directory:
            device = Path(directory) / "1-1"
            device.mkdir()
            (device / "idVendor").write_text("1234\n")
            (device / "idProduct").write_text("abcd\n")
            (device / "product").write_text("USB CAN Adapter\n")
            self.assertEqual(usb_devices(Path(directory)), [{
                "idVendor": "1234",
                "idProduct": "abcd",
                "product": "USB CAN Adapter",
                "sysfs_name": "1-1",
            }])

    def test_module_has_no_can_or_network_configuration_api(self):
        source = (PACKAGE_ROOT / "tocatl_diagnostics" / "pi_inventory.py").read_text()
        forbidden = ("import can", "socket.", ".send(", "ip link set", "set_zero_position")
        self.assertFalse(any(token in source for token in forbidden))


if __name__ == "__main__":
    unittest.main()
