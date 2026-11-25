#!/usr/bin/env python3
"""Self-test of the flight-time mask of flight_check.py on synthetic series. It tests the metric code only; it says
nothing about PX4. Run: python3 tools/manual_sim/selftest_flight_check.py (exit 0 when every case passes).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flight_check  # noqa: E402


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


for case in (case_hold_in_the_middle, case_ground_between_flights, case_sparse_hook_messages):
    case()
    print("PASS", case.__name__)
