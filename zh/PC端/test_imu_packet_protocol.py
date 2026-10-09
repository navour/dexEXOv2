import importlib.util
import os
import struct
import sys
import unittest


_CORE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "IMU API", "multi_imu_core.py")
_SPEC = importlib.util.spec_from_file_location(
    "packet_test_multi_imu_core", _CORE_PATH)
core = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = core
_SPEC.loader.exec_module(core)


class ImuPacketProtocolTest(unittest.TestCase):
    def test_extended_packet(self):
        packet = struct.pack(
            "<BBH4hI3h",
            0x01, 0x01, 65534,
            32767, 0, -16384, 8192,
            0xFFFFFF00,
            123, -456, 789,
        )
        decoded = core.decode_rotation_packet(packet)
        self.assertEqual(decoded["seq"], 65534)
        self.assertEqual(decoded["sensor_timestamp_us"], 0xFFFFFF00)
        self.assertEqual(decoded["gyro_dps"], [12.3, -45.6, 78.9])
        self.assertTrue(decoded["flags"] & 0x01)
        self.assertAlmostEqual(decoded["quat"][0], 1.0)

    def test_legacy_packet(self):
        packet = struct.pack(
            "<BBH4h", 0x01, 0, 7, 32767, 0, 0, 0)
        decoded = core.decode_rotation_packet(packet)
        self.assertEqual(decoded["seq"], 7)
        self.assertIsNone(decoded["sensor_timestamp_us"])
        self.assertIsNone(decoded["gyro_dps"])

    def test_invalid_packet(self):
        self.assertIsNone(core.decode_rotation_packet(b"\x02bad"))


if __name__ == "__main__":
    unittest.main()
