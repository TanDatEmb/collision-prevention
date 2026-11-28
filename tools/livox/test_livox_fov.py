import binascii
import struct
import unittest
import zlib

from livox_fov import decode_reply


def reply(payload, sequence=77):
    header = struct.pack("<BBHIHBB6x", 0xAA, 0, 24 + len(payload), sequence, 0x0101, 1, 1)
    return header + struct.pack("<HI", binascii.crc_hqx(header, 0xFFFF), zlib.crc32(payload)) + payload


class FovReplyTest(unittest.TestCase):
    def test_decodes_real_mid360s_reply_with_disabled_limits(self):
        payload = bytes.fromhex(
            "000300 "
            "15001400 0000000000000000f9ffffff3400000000000000 "
            "16001400 0000000000000000f9ffffff3400000000000000 "
            "17000100 00"
        )
        result = decode_reply(reply(payload), 77)
        self.assertEqual(result.get("fov_cfg_en"), 0)
        self.assertEqual(result.get("fov_cfg0"), {
            "yaw_start_deg": 0, "yaw_stop_deg": 0, "pitch_start_deg": -7, "pitch_stop_deg": 52,
        })

    def test_missing_mask_is_not_disabled(self):
        with self.assertRaisesRegex(ValueError, "fov_cfg_en"):
            decode_reply(reply(bytes.fromhex("000000")), 77)

    def test_reply_from_another_serial_is_not_used_for_the_primary(self):
        payload = bytes.fromhex("000200 17000100 00 00801000") + b"ARMCP550031900\x00\x00"
        with self.assertRaisesRegex(ValueError, "primary serial"):
            decode_reply(reply(payload), 77, "OTHER")

    def test_identifies_the_sensor_that_returned_the_fov(self):
        payload = bytes.fromhex("000200 17000100 00 00801000") + b"ARMCP550031900\x00\x00"
        result = decode_reply(reply(payload), 77, "ARMCP550031900")
        self.assertEqual(result.get("serial"), "ARMCP550031900")

    def test_enabled_region_requires_its_bounds(self):
        with self.assertRaisesRegex(ValueError, "fov_cfg1"):
            decode_reply(reply(bytes.fromhex("000100 17000100 02")), 77)

    def test_enabled_region_retains_its_angles(self):
        payload = bytes.fromhex(
            "000200 17000100 02 "
            "16001400 0a0000005a000000fbffffff1400000000000000"
        )
        self.assertEqual(decode_reply(reply(payload), 77), {
            "fov_cfg_en": 2,
            "fov_cfg1": {
                "yaw_start_deg": 10, "yaw_stop_deg": 90, "pitch_start_deg": -5,
                "pitch_stop_deg": 20,
            },
        })

    def test_truncated_parameter_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "truncated"):
            decode_reply(reply(bytes.fromhex("000100 15001400 00")), 77)

    def test_bad_checksum_is_rejected(self):
        packet = reply(bytes.fromhex("000100 17000100 00"))
        with self.assertRaisesRegex(ValueError, "checksum"):
            decode_reply(packet[:-1] + b"\x01", 77)

    def test_stale_sequence_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "header"):
            decode_reply(reply(bytes.fromhex("000100 17000100 00"), sequence=76), 77)


if __name__ == "__main__":
    unittest.main()
