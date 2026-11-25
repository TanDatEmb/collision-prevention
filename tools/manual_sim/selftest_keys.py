#!/usr/bin/env python3
"""Self-test of the mode handling of keys.py with PX4 mocked. It tests the script only; it says nothing about PX4.
Run: python3 tools/manual_sim/selftest_keys.py (exit 0 when every case passes).
"""
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.modules.setdefault("pymavlink", types.SimpleNamespace(mavutil=None))  # keys.py imports it; not used here
import keys  # noqa: E402


def run_takeoff_with_a_press(key, at_sleep):
    """Run the takeoff worker with PX4 mocked; press key at the given sleep call. Returns the commands sent."""
    sent, sleeps, height = [], [0], [0.0]
    stick = keys.Stick()

    def px4(tool, *args, timeout=10):
        sent.append(" ".join(args))
        return ""

    def listen(topic):
        return {"z": "%f" % -height[0]}

    def sleep(_):
        sleeps[0] += 1
        height[0] += 1.0  # climbing
        if sleeps[0] == at_sleep:
            keys.press(stick, key)

    keys.px4, keys.listen = px4, listen
    keys.time = types.SimpleNamespace(sleep=sleep, time=__import__("time").time)
    keys.threading = types.SimpleNamespace(Thread=lambda target, args, daemon: types.SimpleNamespace(start=lambda: target(*args)))
    keys.takeoff(stick)
    return sent


def case_hold_during_takeoff_is_kept():
    sent = run_takeoff_with_a_press("h", 2)
    assert "mode auto:loiter" in sent, sent
    assert "mode posctl" not in sent[sent.index("mode auto:loiter"):], sent


def case_land_during_takeoff_is_kept():
    # one l asks for confirmation, a second within 3 s lands: press twice
    sent = run_takeoff_with_two_l()
    assert "land" in sent, sent
    assert "mode posctl" not in sent[sent.index("land"):], sent


def run_takeoff_with_two_l():
    sent, sleeps, height = [], [0], [0.0]
    stick = keys.Stick()
    keys.px4 = lambda tool, *args, timeout=10: sent.append(" ".join(args)) or ""
    keys.listen = lambda topic: {"z": "%f" % -height[0]}

    def sleep(_):
        sleeps[0] += 1
        height[0] += 1.0
        if sleeps[0] in (2, 3):
            keys.press(stick, "l")

    keys.time = types.SimpleNamespace(sleep=sleep, time=__import__("time").time)
    keys.threading = types.SimpleNamespace(Thread=lambda target, args, daemon: types.SimpleNamespace(start=lambda: target(*args)))
    keys.takeoff(stick)
    return sent


def case_plain_takeoff_ends_in_position():
    sent = run_takeoff_with_a_press("w", 2)  # a stick key is not a mode request
    assert sent[-1] == "mode posctl", sent


for case in (case_hold_during_takeoff_is_kept, case_land_during_takeoff_is_kept, case_plain_takeoff_ends_in_position):
    case()
    print("PASS", case.__name__)
