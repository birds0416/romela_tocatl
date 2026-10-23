from pathlib import Path
from contextlib import redirect_stdout
import io
import sys
import tempfile
import unittest


PACKAGE_ROOT = Path(__file__).parents[1] / "src" / "tocatl_control"
sys.path.insert(0, str(PACKAGE_ROOT))

from tocatl_control.calibration import (  # noqa: E402
    JointCalibration,
    calibration_template,
    document_from_dict,
    document_to_dict,
    expected_joints,
    load_calibration,
    main,
    validate_calibration,
    write_calibration,
)


def valid_document():
    template = calibration_template()
    return document_from_dict({
        **document_to_dict(template),
        "source": "unit test filled calibration",
        "joints": [
            {
                "motor_id": motor_id,
                "leg": leg,
                "joint": joint,
                "bus": bus,
                "sign": 1,
                "zero_offset_rad": 0.0,
                "min_position_rad": -0.5,
                "max_position_rad": 0.5,
                "park_position_rad": 0.0,
                "method": "unit test",
                "notes": "",
            }
            for motor_id, leg, joint, bus in expected_joints()
        ],
    })


class CalibrationTests(unittest.TestCase):
    def test_template_has_all_18_joints_but_is_not_valid_for_arming(self):
        document = calibration_template()
        self.assertEqual(len(document.joints), 18)
        errors = validate_calibration(document)
        self.assertTrue(any("sign must be -1 or 1" in error for error in errors))
        self.assertTrue(any("zero_offset_rad is required" in error for error in errors))

    def test_filled_document_validates(self):
        self.assertEqual(validate_calibration(valid_document()), [])

    def test_missing_motor_is_invalid(self):
        document = valid_document()
        trimmed = document_from_dict({
            **document_to_dict(document),
            "joints": [document_to_dict(document)["joints"][0]],
        })
        errors = validate_calibration(trimmed)
        self.assertTrue(any("missing motor_id 12" in error for error in errors))

    def test_mismatched_bus_is_invalid(self):
        data = document_to_dict(valid_document())
        data["joints"][0]["bus"] = "can0"
        errors = validate_calibration(document_from_dict(data))
        self.assertIn("motor 11 bus='can0'; expected 'can1'", errors)

    def test_limits_are_checked(self):
        data = document_to_dict(valid_document())
        data["joints"][0]["min_position_rad"] = 1.0
        data["joints"][0]["max_position_rad"] = -1.0
        errors = validate_calibration(document_from_dict(data))
        self.assertIn("motor 11 min_position_rad must be < max_position_rad", errors)

    def test_write_and_load_round_trip(self):
        document = valid_document()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration.json"
            write_calibration(path, document)
            loaded = load_calibration(path)
        self.assertEqual(document, loaded)

    def test_cli_template_write_is_incomplete_by_design(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "template.json"
            with redirect_stdout(io.StringIO()):
                self.assertEqual(main(["--write-template", str(path)]), 0)
                self.assertNotEqual(main([str(path)]), 0)

    def test_module_has_no_motor_command_api(self):
        source = (PACKAGE_ROOT / "tocatl_control" / "calibration.py").read_text()
        forbidden = ("import can", ".send(", ".enable(", ".control(", "set_zero_position")
        self.assertFalse(any(token in source for token in forbidden))


if __name__ == "__main__":
    unittest.main()
