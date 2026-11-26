#!/usr/bin/env python3
"""Repository contract tests for the live dashboard telemetry path."""

import contextlib
import importlib.util
import os
import pathlib
import signal
import subprocess
import sys
import time
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

    def test_browser_contract_keeps_live_controls_and_leveling_coverage_view(self):
        html = DASHBOARD_HTML.read_text()
        for text in (
            "Connect",
            "Disconnect",
            'EventSource("/events")',
            "Vertical coverage",
            "Sensor FOV",
            "Current view",
            "Horizon",
            "Band",
            "Coverage unavailable",
            "Up",
            "Down",
            "speedUnit",
            "hook-age",
            "Limit",
            "Far / clear",
        ):
            self.assertIn(text, html)
        self.assertNotIn("No ray pairs", html)


@contextlib.contextmanager
def running_dashboard():
    """A process that looks like a running dashboard to the launcher; it serves nothing."""
    process = subprocess.Popen(["bash", "-c", "exec -a 'python3 tools/dashboard/bridge.py' sleep 60"])
    try:
        time.sleep(0.3)
        yield
    finally:
        process.kill()
        process.wait()


def launch(**extra):
    """Run the manual-sim launcher up to its file checks: PX4_BUILD points nowhere, so nothing is started."""
    env = dict(os.environ, PX4_BUILD="/nonexistent-px4-build", **extra)
    return subprocess.run(["bash", str(ROOT / "tools/manual_sim/fly.sh")], env=env, capture_output=True, text=True,
                          timeout=30, check=False).stdout


class LauncherContractTests(unittest.TestCase):
    def setUp(self):
        if "Another simulation is running" in launch(DASHBOARD="0"):
            self.skipTest("a simulator is running on this machine")

    def test_a_running_dashboard_does_not_block_the_simulator(self):
        # make dashboard serves and waits for SITL: starting or restarting the simulator must work next to it
        with running_dashboard():
            output = launch(DASHBOARD="0")
        self.assertIn("missing: /nonexistent-px4-build/bin/px4", output)

    def test_a_second_dashboard_is_refused(self):
        with running_dashboard():
            output = launch(DASHBOARD="1")
        self.assertIn("A dashboard is already running", output)


@unittest.skipUnless(importlib.util.find_spec("rclpy"), "needs a sourced ROS 2 workspace")
class ShutdownContractTests(unittest.TestCase):
    def test_a_plain_kill_stops_the_bridge(self):
        # rclpy installs its own SIGTERM handler, which stops only the ROS side and leaves the server running
        process = subprocess.Popen([sys.executable, str(ROOT / "tools/dashboard" / "bridge.py"), "--port", "0"],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            time.sleep(3)
            self.assertIsNone(process.poll(), "the bridge did not start")
            process.send_signal(signal.SIGTERM)
            process.wait(timeout=6)
        finally:
            process.kill()
            process.wait()


if __name__ == "__main__":
    unittest.main()
