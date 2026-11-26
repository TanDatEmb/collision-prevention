#!/usr/bin/env python3
"""Unit tests for the SITL dashboard bridge."""

import json
import math
import queue
import types
import unittest

from bridge import Px4Listener, SnapshotStore, _diagnostics_payload, parse_listener_output, sse_event


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

    def test_listener_rejects_metadata_only_error_output(self):
        def runner(*_args, **_kwargs):
            return types.SimpleNamespace(
                returncode=0,
                stdout=(
                    "TOPIC: cp_brake_constraints\n"
                    "ERROR [listener] topic cp_brake_constraints too large (608 > 512)\n"
                ),
            )

        listener = Px4Listener(binary=__file__, runner=runner)

        self.assertIsNone(listener.poll("cp_brake_constraints"))

    def test_listener_rejects_partial_constraint_arrays(self):
        def runner(*_args, **_kwargs):
            return types.SimpleNamespace(
                returncode=0,
                stdout="timestamp: 1234567\ncaps_m_s[0]: 4.25\n",
            )

        listener = Px4Listener(binary=__file__, runner=runner)

        self.assertIsNone(listener.poll("cp_brake_constraints"))


class SnapshotStoreTests(unittest.TestCase):
    @staticmethod
    def coverage_payload(**overrides):
        payload = {
            "timestamp": 2_000_000,
            "sample_timestamp": 1_900_000,
            "mount_roll_rad": 0.0,
            "mount_pitch_down_rad": math.radians(10.0),
            "mount_yaw_left_rad": 0.0,
            "sensor_elevation_min_rad": math.radians(-7.0),
            "sensor_elevation_max_rad": math.radians(52.0),
            "height_up_m": 0.25,
            "height_down_m": 0.60,
            "attitude_allowance_rad": 0.0157,
            "required_rays": 5,
            "packet_continuity_valid": True,
            "up_ray_count": [5] * 72,
            "down_ray_count": [5] * 72,
            "rear_only": [False] * 72,
            "coverage_ok": [True] * 72,
        }
        payload.update(overrides)
        return payload

    def test_leveling_view_uses_mount_and_sensor_fov_at_hover(self):
        store = SnapshotStore(clock=lambda: 100.0)
        store.update_leveling_coverage(self.coverage_payload())
        store.update_odometry(
            {
                "timestamp": 1_000_000,
                "position": [0.0, 0.0, -4.0],
                "velocity": [0.0, 0.0, 0.0],
                "q": [1.0, 0.0, 0.0, 0.0],
            }
        )

        leveling = store.snapshot()["leveling"]

        self.assertAlmostEqual(leveling["view_by_bin"][0]["min_deg"], -17.0, places=3)
        self.assertAlmostEqual(leveling["view_by_bin"][0]["max_deg"], 42.0, places=3)
        self.assertEqual(leveling["coverage"]["bins"][0]["state"], "Covered")
        self.assertEqual(leveling["band"]["height_down_m"], 0.60)

    def test_leveling_view_moves_with_vehicle_pitch(self):
        store = SnapshotStore(clock=lambda: 100.0)
        store.update_leveling_coverage(self.coverage_payload())
        pitch = math.radians(10.0)
        store.update_odometry(
            {
                "timestamp": 1_000_000,
                "position": [0.0, 0.0, -4.0],
                "velocity": [0.0, 0.0, 0.0],
                "q": [math.cos(pitch / 2.0), 0.0, math.sin(pitch / 2.0), 0.0],
            }
        )

        view = store.snapshot()["leveling"]["view_by_bin"][0]

        self.assertAlmostEqual(view["min_deg"], -7.0, places=3)
        self.assertAlmostEqual(view["max_deg"], 52.0, places=3)

    def test_incomplete_packet_is_not_labelled_clear(self):
        store = SnapshotStore(clock=lambda: 100.0)
        store.update_leveling_coverage(
            self.coverage_payload(
                packet_continuity_valid=False,
                down_ray_count=[2] * 72,
                coverage_ok=[False] * 72,
            )
        )

        coverage = store.snapshot()["leveling"]["coverage"]

        self.assertFalse(coverage["packet_continuity_valid"])
        self.assertEqual(coverage["bins"][0]["state"], "Unknown")
        self.assertEqual(coverage["bins"][0]["down_ray_count"], 2)

    def test_coverage_decision_is_derived_when_optional_flags_are_absent(self):
        store = SnapshotStore(clock=lambda: 100.0)
        payload = self.coverage_payload()
        payload.pop("coverage_ok")
        store.update_leveling_coverage(payload)

        self.assertTrue(store.snapshot()["leveling"]["coverage"]["bins"][0]["coverage_ok"])

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
        self.assertEqual(snapshot["leveling"]["status"], "pose_only")

    def test_caps_do_not_regress_when_listener_returns_an_older_sample(self):
        store = SnapshotStore(clock=lambda: 100.0)
        store.update_caps({"timestamp": 20, "caps_m_s": [2.0] * 72})
        store.update_caps({"timestamp": 10, "caps_m_s": [0.0] * 72})

        caps = store.snapshot()["caps"]
        self.assertEqual(caps["timestamp_us"], 20)
        self.assertEqual(caps["caps_m_s"][0], 2.0)

    def test_older_caps_sample_does_not_refresh_source_age(self):
        now = [100.0]
        store = SnapshotStore(clock=lambda: now[0], source_timeout_s=2.0)
        store.update_caps({"timestamp": 20, "caps_m_s": [2.0] * 72})
        now[0] = 103.0

        store.update_caps({"timestamp": 10, "caps_m_s": [0.0] * 72})

        self.assertEqual(store.snapshot()["sources"]["caps"]["state"], "stale")

    def test_caps_follow_px4_after_a_restart(self):
        # PX4 timestamps count from boot: after a restart every sample is smaller than the stored one
        now = [100.0]
        store = SnapshotStore(clock=lambda: now[0], source_timeout_s=2.0)
        store.update_caps({"timestamp": 900_000_000, "caps_m_s": [1.0] * 72})
        now[0] = 115.0
        store.update_caps({"timestamp": 20_000_000, "caps_m_s": [9.0] * 72})
        now[0] = 115.05
        store.update_caps({"timestamp": 20_050_000, "caps_m_s": [8.0] * 72})

        snapshot = store.snapshot()
        self.assertEqual(snapshot["caps"]["timestamp_us"], 20_050_000)
        self.assertEqual(snapshot["caps"]["caps_m_s"][0], 8.0)
        self.assertEqual(snapshot["sources"]["caps"]["state"], "live")

    def test_hook_follows_px4_after_a_restart(self):
        now = [100.0]
        store = SnapshotStore(clock=lambda: now[0], source_timeout_s=2.0)
        store.update_hook({"timestamp": 900_000_000, "action": 3})
        now[0] = 115.0
        store.update_hook({"timestamp": 20_000_000, "action": 2})
        now[0] = 115.1
        store.update_hook({"timestamp": 20_100_000, "action": 2})

        snapshot = store.snapshot()
        self.assertEqual(snapshot["hook"]["timestamp_us"], 20_100_000)
        self.assertEqual(snapshot["hook"]["action"], 2)
        self.assertEqual(snapshot["sources"]["hook"]["state"], "live")

    def test_a_repeated_older_sample_is_not_a_restart(self):
        # the listener of a topic that stopped publishing returns the same sample again and again
        now = [100.0]
        store = SnapshotStore(clock=lambda: now[0], source_timeout_s=2.0)
        store.update_caps({"timestamp": 20, "caps_m_s": [2.0] * 72})
        for later in (103.0, 103.2, 103.4):
            now[0] = later
            store.update_caps({"timestamp": 10, "caps_m_s": [0.0] * 72})

        snapshot = store.snapshot()
        self.assertEqual(snapshot["caps"]["timestamp_us"], 20)
        self.assertEqual(snapshot["sources"]["caps"]["state"], "stale")

    def test_advancing_older_samples_wait_until_the_stored_one_is_stale(self):
        # delayed reads of the same boot must not pull the display back while newer data is fresh
        now = [100.0]
        store = SnapshotStore(clock=lambda: now[0], source_timeout_s=2.0)
        store.update_hook({"timestamp": 100, "action": 3})
        store.update_hook({"timestamp": 10, "action": 2})
        store.update_hook({"timestamp": 11, "action": 2})
        self.assertEqual(store.snapshot()["hook"]["timestamp_us"], 100)

        now[0] = 103.0
        store.update_hook({"timestamp": 12, "action": 2})
        self.assertEqual(store.snapshot()["hook"]["timestamp_us"], 12)

    def test_snapshot_event_recomputes_stale_source_state(self):
        now = [100.0]
        store = SnapshotStore(clock=lambda: now[0], source_timeout_s=2.0)
        store.update_map({"timestamp": 1, "distances": [1] * 72})
        now[0] = 103.0

        event = store.snapshot_event()

        self.assertEqual(event["event"], "snapshot")
        self.assertEqual(json.loads(event["data"])["connection"], "stale")

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

    def test_subscriber_coalesces_high_rate_updates(self):
        now = [100.0]
        store = SnapshotStore(clock=lambda: now[0], event_interval_s=0.05)
        subscriber = store.subscribe()
        store.update_hook({"timestamp": 1, "action": 3, "active_sector": 0})
        first = json.loads(subscriber.get(timeout=0.2)["data"])

        now[0] = 100.01
        store.update_hook({"timestamp": 2, "action": 4, "active_sector": 1})
        with self.assertRaises(queue.Empty):
            subscriber.get_nowait()

        now[0] = 100.06
        store.update_hook({"timestamp": 3, "action": 2, "active_sector": 2})
        latest = json.loads(subscriber.get(timeout=0.2)["data"])

        self.assertEqual(first["hook"]["timestamp_us"], 1)
        self.assertEqual(latest["hook"]["timestamp_us"], 3)
        store.unsubscribe(subscriber)


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
