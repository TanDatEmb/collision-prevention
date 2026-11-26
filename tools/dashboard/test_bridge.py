#!/usr/bin/env python3
"""Unit tests for the SITL dashboard bridge."""

import json
import queue
import unittest

from bridge import SnapshotStore, _diagnostics_payload, parse_listener_output, sse_event


class ListenerParserTests(unittest.TestCase):
    def test_parses_scalars_and_indexed_arrays_from_px4_listener(self):
        output = """
 cp_brake_constraints
  timestamp: 1234567 (0.01 seconds ago)
  sample_timestamp: 1234000
  heading_valid: true
  valid: false
  caps_m_s[0]: 4.25
  caps_m_s[1]: 0.0
  caps_m_s[71]: 5.0
  reason_flags: 64
 """

        parsed = parse_listener_output(output)

        self.assertEqual(parsed["timestamp"], 1234567)
        self.assertEqual(parsed["sample_timestamp"], 1234000)
        self.assertTrue(parsed["heading_valid"])
        self.assertFalse(parsed["valid"])
        self.assertEqual(parsed["caps_m_s"], {0: 4.25, 1: 0.0, 71: 5.0})
        self.assertEqual(parsed["reason_flags"], 64)

    def test_parses_bracketed_arrays_and_nan(self):
        parsed = parse_listener_output(
            "  vel_sp_in: [1.0, -2.0]\n  acc_sp_in: [nan, 0.25]\n"
        )

        self.assertEqual(parsed["vel_sp_in"], [1.0, -2.0])
        self.assertEqual(parsed["acc_sp_in"][0], "nan")
        self.assertEqual(parsed["acc_sp_in"][1], 0.25)


class SnapshotStoreTests(unittest.TestCase):
    def test_snapshot_reports_ros_connection_without_inventing_caps(self):
        now = [100.0]
        store = SnapshotStore(clock=lambda: now[0], source_timeout_s=2.0)
        store.update_map(
            {
                "timestamp": 1_000_000,
                "frame": 12,
                "increment": 5.0,
                "angle_offset": 0.0,
                "distances": [65535] * 72,
                "min_distance": 10,
                "max_distance": 4000,
            }
        )
        store.update_odometry(
            {
                "timestamp": 1_000_000,
                "position": [0.0, 0.0, -4.0],
                "velocity": [2.0, 0.0, 0.0],
                "q": [1.0, 0.0, 0.0, 0.0],
            }
        )

        snapshot = store.snapshot()

        self.assertEqual(snapshot["connection"], "connected")
        self.assertEqual(snapshot["map"]["distances_cm"], [65535] * 72)
        self.assertIsNone(snapshot["caps"])
        self.assertAlmostEqual(snapshot["aircraft"]["speed_m_s"], 2.0)
        self.assertAlmostEqual(snapshot["aircraft"]["height_m"], 4.0)
        self.assertEqual(snapshot["leveling"]["status"], "no_ray_pairs")

    def test_source_ages_make_stale_connection_explicit(self):
        now = [100.0]
        store = SnapshotStore(clock=lambda: now[0], source_timeout_s=2.0)
        store.update_map({"timestamp": 1, "distances": [1] * 72})
        now[0] = 103.0

        snapshot = store.snapshot()

        self.assertEqual(snapshot["connection"], "stale")
        self.assertGreater(snapshot["sources"]["map"]["age_s"], 2.0)

    def test_subscriber_receives_json_sse_event(self):
        store = SnapshotStore(clock=lambda: 10.0)
        subscriber = store.subscribe()
        store.update_hook({"timestamp": 42, "action": 3, "active_sector": 0})

        event = subscriber.get(timeout=0.2)

        self.assertEqual(event["event"], "snapshot")
        decoded = json.loads(event["data"])
        self.assertEqual(decoded["hook"]["action"], 3)
        store.unsubscribe(subscriber)
        with self.assertRaises(queue.Empty):
            subscriber.get_nowait()


class SseTests(unittest.TestCase):
    def test_sse_event_has_json_payload_and_two_newlines(self):
        result = sse_event("snapshot", {"connection": "waiting"})

        self.assertTrue(result.startswith("event: snapshot\ndata: "))
        self.assertTrue(result.endswith("\n\n"))
        self.assertEqual(json.loads(result.split("data: ", 1)[1].strip())["connection"], "waiting")


class DiagnosticsTests(unittest.TestCase):
    def test_ros_uint8_bytes_are_normalized(self):
        class Value:
            key = "frames"
            value = "12"

        class Status:
            name = "obstacle_sector_node"
            level = b"\x01"
            message = "running"
            values = [Value()]

        class Message:
            status = [Status()]

        payload = _diagnostics_payload(Message())

        self.assertEqual(payload["statuses"][0]["level"], 1)


if __name__ == "__main__":
    unittest.main()
