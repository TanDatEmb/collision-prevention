#!/usr/bin/env python3
"""Repository contract tests for the live dashboard telemetry path."""

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
PX4_ROOT = ROOT / "firmware" / "PX4-Autopilot"
DDS_YAML = PX4_ROOT / "src/modules/uxrce_dds_client/dds_topics.yaml"
DDS_TEMPLATE = PX4_ROOT / "src/modules/uxrce_dds_client/dds_topics.h.em"
ROS_PACKAGE = ROOT / "ros2_ws/src/collision_prevention_msgs"
DASHBOARD_HTML = ROOT / "tools/dashboard/index.html"
MAKEFILE = ROOT / "Makefile"


class DdsContractTests(unittest.TestCase):
    def test_dashboard_messages_mirror_px4_uorb_fields(self):
        for name in ("CpBrakeConstraints", "CpBrakeHook"):
            ros_message = ROS_PACKAGE / "msg" / f"{name}.msg"
            px4_message = PX4_ROOT / "msg" / f"{name}.msg"
            self.assertTrue(ros_message.is_file(), ros_message)
            self.assertEqual(ros_message.read_text(), px4_message.read_text())

    def test_dds_topics_export_the_two_monitor_topics(self):
        text = DDS_YAML.read_text()
        self.assertIn("/fmu/out/cp_brake_constraints", text)
        self.assertIn("/fmu/out/cp_brake_hook", text)
        self.assertIn("collision_prevention_msgs::msg::CpBrakeConstraints", text)
        self.assertIn("collision_prevention_msgs::msg::CpBrakeHook", text)

    def test_dds_buffer_covers_constraint_payload(self):
        template = DDS_TEMPLATE.read_text()
        self.assertRegex(template, r"max_topic_size\s*=\s*(?:7[0-9]{2}|[89][0-9]{2}|[1-9][0-9]{3,})")

    def test_launch_contract_exposes_dashboard_targets(self):
        makefile = MAKEFILE.read_text()
        self.assertIn("dashboard:", makefile)
        self.assertIn("sim-dashboard:", makefile)
        self.assertIn("tools/dashboard/bridge.py", makefile)

    def test_browser_contract_keeps_live_controls_and_explicit_pair_empty_state(self):
        html = DASHBOARD_HTML.read_text()
        for text in (
            "Connect",
            "Disconnect",
            'EventSource("/events")',
            "No ray pairs",
            "Pose live",
            "speedUnit",
            "hook-age",
            "Limit",
            "Far / clear",
        ):
            self.assertIn(text, html)


if __name__ == "__main__":
    unittest.main()
