#!/usr/bin/env python3
"""Self-test of flight_check.py on synthetic series and topic fixtures. It says nothing about PX4.
Run: python3 tools/manual_sim/selftest_flight_check.py (exit 0 when every case passes).
"""
import contextlib
import io
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flight_check  # noqa: E402


class FakeTopic:
    def __init__(self, name, data):
        self.name = name
        self.data = data
        self.multi_id = 0


class FakeULog:
    def __init__(self, topics):
        self.data_list = topics


def fake_flight_topics(include_hook=True, include_constraints=True, include_other_px4=True):
    t = np.arange(0.0, 3.0, 0.1)
    ts = (t * 1e6).astype(np.uint64)
    topics = [FakeTopic("vehicle_local_position", {
        "timestamp": ts, "z": np.full_like(t, -5.0),
        "vx": np.full_like(t, 1.0), "vy": np.zeros_like(t),
        "z_valid": np.ones_like(t, dtype=bool), "v_xy_valid": np.ones_like(t, dtype=bool),
    })]
    if include_other_px4:
        topics.extend([
            FakeTopic("vehicle_attitude", {
                "timestamp": ts, "q[0]": np.ones_like(t),
                "q[1]": np.zeros_like(t), "q[2]": np.zeros_like(t), "q[3]": np.zeros_like(t),
            }),
            FakeTopic("vehicle_attitude_setpoint", {
                "timestamp": ts, "q_d[0]": np.ones_like(t),
                "q_d[1]": np.zeros_like(t), "q_d[2]": np.zeros_like(t), "q_d[3]": np.zeros_like(t),
            }),
            FakeTopic("vehicle_angular_velocity", {
                "timestamp": ts, "xyz[0]": np.zeros_like(t), "xyz[1]": np.zeros_like(t),
            }),
            FakeTopic("manual_control_setpoint", {
                "timestamp": ts, "pitch": np.zeros_like(t), "roll": np.zeros_like(t),
                "valid": np.ones_like(t, dtype=bool),
            }),
        ])
    if include_hook:
        topics.append(FakeTopic("cp_brake_hook", {
            "timestamp": ts, "action": np.full_like(t, 2, dtype=np.uint8),
            "acc_sp_in[0]": np.zeros_like(t), "acc_sp_in[1]": np.zeros_like(t),
        }))
    if include_constraints:
        topics.append(FakeTopic("cp_brake_constraints", {
            "timestamp": ts,
            **{"caps_m_s[%d]" % sector: np.full_like(t, 10.0) for sector in range(72)},
        }))
    return topics


def run_fake_check(topics):
    original_ulog = flight_check.ULog
    flight_check.ULog = lambda _path: FakeULog(topics)
    output = io.StringIO()
    try:
        with contextlib.redirect_stdout(output):
            flight_check.check("cp-off.ulg")
    finally:
        flight_check.ULog = original_ulog
    return output.getvalue()


def assert_cp_missing_reports_baseline(topics, missing):
    text = run_fake_check(topics)
    assert "max altitude 5.0 m" in text, text
    assert "max speed 1.0 m/s" in text, text
    assert "missing %s" % missing in text, text
    assert "CP-specific metrics: N/A" in text, text


def case_missing_hook_reports_baseline():
    assert_cp_missing_reports_baseline(fake_flight_topics(include_hook=False), "cp_brake_hook")


def case_missing_constraints_reports_baseline():
    assert_cp_missing_reports_baseline(fake_flight_topics(include_constraints=False), "cp_brake_constraints")


def case_missing_cp_topics_need_only_position():
    text = run_fake_check(fake_flight_topics(include_hook=False, include_constraints=False, include_other_px4=False))
    assert "max altitude 5.0 m" in text, text
    assert "max speed 1.0 m/s" in text, text
    assert "CP-specific metrics: N/A" in text, text


def case_cp_off_reports_available_px4_metrics():
    text = run_fake_check(fake_flight_topics(include_hook=False, include_constraints=False))
    assert "stick held but no motion (>= 0.8 s):" in text, text
    assert "tilt above 20 deg:" in text, text
    assert "tilt command steps above 10 deg:" in text, text


def case_invalid_manual_control_is_na():
    topics = fake_flight_topics(include_hook=False, include_constraints=False)
    for topic in topics:
        if topic.name == "manual_control_setpoint":
            topic.data["valid"][:] = False
    text = run_fake_check(topics)
    assert "stick held but no motion (>= 0.8 s): N/A (no valid samples" in text, text


def case_stick_episodes_respect_manual_coverage():
    topics = fake_flight_topics(include_hook=False, include_constraints=False)
    topics[0].data["vx"][:] = 0.0
    for topic in topics:
        if topic.name == "manual_control_setpoint":
            topic.data["valid"][:] = False
            topic.data["valid"][10:12] = True
            topic.data["pitch"][10:12] = 1.0
    text = run_fake_check(topics)
    assert "stick held but no motion (>= 0.8 s): 0 episodes, 0.0 s per minute" in text, text


def case_missing_velocity_is_na_for_stick_metric():
    topics = fake_flight_topics(include_hook=False, include_constraints=False)
    topics[0].data["v_xy_valid"][:] = False
    text = run_fake_check(topics)
    assert "stick held but no motion (>= 0.8 s): N/A (no valid stick and velocity samples" in text, text


def case_cp_off_metrics_respect_position_time_range():
    topics = fake_flight_topics(include_hook=False, include_constraints=False)
    for topic in topics:
        if topic.name == "vehicle_attitude":
            topic.data["timestamp"] = topic.data["timestamp"] + 10_000_000
            topic.data["q[0]"][:] = 0.0
            topic.data["q[1]"][:] = 1.0
        elif topic.name == "vehicle_attitude_setpoint":
            topic.data["timestamp"] = topic.data["timestamp"] + 10_000_000
            topic.data["q_d[0]"][:] = 0.0
            topic.data["q_d[1]"][:] = 1.0
    text = run_fake_check(topics)
    assert "tilt above 20 deg: N/A" in text, text
    assert "tilt command steps above 10 deg: N/A" in text, text


def case_command_without_sample_pair_is_na():
    topics = fake_flight_topics(include_hook=False, include_constraints=False)
    for topic in topics:
        if topic.name == "vehicle_attitude_setpoint":
            for field in topic.data:
                topic.data[field] = topic.data[field][:1]
    text = run_fake_check(topics)
    assert "tilt command steps above 10 deg: N/A" in text, text


def case_no_valid_altitude_is_distinguished():
    topics = fake_flight_topics(include_hook=False, include_constraints=False)
    topics[0].data["z_valid"][:] = False
    text = run_fake_check(topics)
    assert "no valid altitude samples" in text, text


def case_cp_on_metrics_respect_position_time_range():
    topics = fake_flight_topics()
    for topic in topics:
        if topic.name == "vehicle_attitude":
            topic.data["timestamp"] = topic.data["timestamp"] + 10_000_000
            topic.data["q[0]"][:] = 0.0
            topic.data["q[1]"][:] = 1.0
        elif topic.name == "vehicle_attitude_setpoint":
            topic.data["timestamp"] = topic.data["timestamp"] + 10_000_000
            topic.data["q_d[0]"][:] = 0.0
            topic.data["q_d[1]"][:] = 1.0
    text = run_fake_check(topics)
    assert "tilt above 20 deg: N/A" in text, text
    assert "tilt command steps above 10 deg: N/A" in text, text


def case_invalid_px4_samples_are_excluded():
    topics = fake_flight_topics(include_hook=False, include_constraints=False)
    local_position = topics[0].data
    local_position["z"][0] = -100.0
    local_position["z_valid"][0] = False
    local_position["v_xy_valid"][0] = False
    local_position["vx"][0] = 123.0
    text = run_fake_check(topics)
    assert "max altitude 5.0 m" in text, text
    assert "max speed 1.0 m/s" in text, text


def case_no_airborne_interval_still_reports_altitude():
    topics = fake_flight_topics(include_hook=False, include_constraints=False)
    topics[0].data["z"] = np.full(30, -0.8)
    text = run_fake_check(topics)
    assert "max altitude 0.8 m" in text, text
    assert "max speed N/A" in text, text


def case_nonfinite_speed_is_ignored():
    topics = fake_flight_topics(include_hook=False, include_constraints=False)
    topics[0].data["vx"] = np.array([1.0, np.nan, 2.0] + [1.0] * 27)
    text = run_fake_check(topics)
    assert "max speed 2.0 m/s" in text, text


def case_cp_on_without_manual_control_reports_other_metrics():
    topics = [topic for topic in fake_flight_topics() if topic.name != "manual_control_setpoint"]
    text = run_fake_check(topics)
    assert "CP-specific metrics" not in text, text
    assert "stick held but no motion (>= 0.8 s): N/A" in text, text
    assert "forward sectors free -> 0 in one step" in text, text


def case_cp_on_without_velocity_reports_na():
    topics = fake_flight_topics()
    topics[0].data["v_xy_valid"][:] = False
    text = run_fake_check(topics)
    assert "stick held but no motion (>= 0.8 s): N/A (no valid velocity samples" in text, text


def case_cp_on_without_airborne_interval_reports_baseline():
    topics = fake_flight_topics()
    topics[0].data["z"] = np.full(30, -0.8)
    text = run_fake_check(topics)
    assert "no time airborne with cp_brake enforcing" in text, text
    assert "max altitude 0.8 m" in text, text
    assert "max speed N/A" in text, text


def case_cp_on_nonfinite_speed_is_ignored():
    topics = fake_flight_topics()
    topics[0].data["vx"] = np.array([1.0, np.nan, 2.0] + [1.0] * 27)
    text = run_fake_check(topics)
    assert "max speed 2.0 m/s" in text, text
    assert "max speed nan m/s" not in text, text


def case_hold_in_the_middle():
    # Position 0-60 s, Hold 60-120 s (hook transparent), Position again 120-180 s, airborne throughout
    t = np.arange(0.0, 180.0, 0.1)
    hook_t = np.arange(0.0, 180.0, 0.1)
    action = np.where((hook_t >= 60.0) & (hook_t < 120.0), 0, 2)
    lp_t = t
    altitude = np.full_like(t, 5.0)
    mask = flight_check.active_mask(t, hook_t, action, lp_t, altitude)
    minutes = np.sum(np.diff(t) * mask[:-1]) / 60.0
    assert abs(minutes - 2.0) < 0.01, minutes
    assert not mask[(t > 61.0) & (t < 119.0)].any()


def case_ground_between_flights():
    # hook enforcing all along, on the ground between 50 and 100 s
    t = np.arange(0.0, 150.0, 0.1)
    action = np.full_like(t, 3)
    altitude = np.where((t >= 50.0) & (t < 100.0), 0.0, 5.0)
    mask = flight_check.active_mask(t, t, action, t, altitude)
    assert not mask[(t > 51.0) & (t < 99.0)].any()
    assert mask[(t > 1.0) & (t < 49.0)].all() and mask[t > 101.0].all()


def case_sparse_hook_messages():
    # the hook publishes at 10 Hz and on every change: between two messages the last one holds
    t = np.arange(0.0, 10.0, 0.01)
    hook_t = np.array([0.0, 0.1, 5.0, 5.1])
    action = np.array([2, 2, 0, 0])
    mask = flight_check.active_mask(t, hook_t, action, t, np.full_like(t, 5.0))
    assert mask[(t > 0.2) & (t < 4.99)].all()
    assert not mask[t > 5.01].any()


def case_episode_open_at_the_end():
    # stopped while still held against an obstacle: the last episode runs to the end of the log
    eps = flight_check.episodes([False, True, True, True, True], [0.0, 1.0, 2.0, 3.0, 4.0], 0.8)
    assert eps == [(1.0, 3.0)], eps


def case_window_after_the_last_sample():
    # a sector drop at or after the last attitude sample leaves an empty window: no crash, no value
    t = np.array([0.0, 1.0, 2.0])
    v = np.array([5.0, 10.0, 15.0])
    assert flight_check.window_max(t, v, 1.5, 3.0) == 15.0
    assert np.isnan(flight_check.window_max(t, v, 2.5, 4.0))


for case in (case_hold_in_the_middle, case_ground_between_flights, case_sparse_hook_messages, case_episode_open_at_the_end,
             case_window_after_the_last_sample, case_missing_hook_reports_baseline, case_missing_constraints_reports_baseline,
             case_missing_cp_topics_need_only_position, case_cp_off_reports_available_px4_metrics,
             case_invalid_manual_control_is_na, case_stick_episodes_respect_manual_coverage,
             case_missing_velocity_is_na_for_stick_metric, case_cp_off_metrics_respect_position_time_range,
             case_command_without_sample_pair_is_na, case_no_valid_altitude_is_distinguished,
             case_cp_on_metrics_respect_position_time_range, case_invalid_px4_samples_are_excluded,
             case_no_airborne_interval_still_reports_altitude,
             case_nonfinite_speed_is_ignored, case_cp_on_without_manual_control_reports_other_metrics,
             case_cp_on_without_velocity_reports_na,
             case_cp_on_without_airborne_interval_reports_baseline, case_cp_on_nonfinite_speed_is_ignored):
    case()
    print("PASS", case.__name__)
