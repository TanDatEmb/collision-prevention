#!/usr/bin/env python3
"""Status screen and keys for the manual-flight SITL (bench only, tools/manual_sim). By default the stick is QGroundControl's virtual
joystick; with JOY=keys this script sends MAVLink MANUAL_CONTROL at 20 Hz from the keyboard instead.

KEYS="3:t,35:w,..." replays key presses at those seconds (used to test this script without a keyboard).
"""
import math
import os
import re
import select
import shutil
import subprocess
import sys
import termios
import threading
import time
import tty

from pymavlink import mavutil

BIN = os.environ.get("PX4_BIN", "")  # set by fly.sh: the bin/ of the PX4 SITL build
STEP = 250  # stick step per key press (out of 1000)
QGC = os.environ.get("JOY", "qgc") != "keys"
MODES = {2: "Position", 3: "MISSION", 4: "Hold", 5: "RTL", 17: "Takeoff", 18: "Land", 0: "Manual"}
STATES = {0: "OFF", 1: "observer only", 2: "not ready", 3: "clear", 4: "limiting", 5: "SAFE STOP"}
HELP_QGC = """
FLY with the QGroundControl virtual joystick (Fly view):
  left stick: up/down + yaw    right stick: forward/back + left/right
KEYS in this terminal: t take off   p Position   h Hold   u clear SAFE STOP
  l l land   x quit
Stay in the Fly view while flying (no stick means PX4 goes to Hold), and do not
change mode next to an obstacle (issue #209).
"""
HELP = """
KEYS (each press moves the stick 25 %; the stick does NOT return to centre)
  t take off   p Position   h Hold   SPACE centre the stick
  w/s forward/back   a/d left/right   q/e yaw   r/f up/down
  u clear SAFE STOP (away from obstacles)   l l land   x quit
Do not change mode next to an obstacle (issue #209).
"""


def px4(tool, *args, timeout=10):
    try:
        return subprocess.run([BIN + "/px4-" + tool, *args], capture_output=True, text=True, timeout=timeout).stdout
    except (OSError, subprocess.SubprocessError):
        return ""


def listen(topic):
    values = {}
    for line in px4("listener", topic, "-n", "1", timeout=3).splitlines():
        m = re.match(r"\s+(\w+(?:\[\d+\])?): (.+)", line)
        if m:
            values[m.group(1)] = m.group(2).strip()
    return values


def num(d, key, default=float("nan")):
    try:
        return float(d[key].split()[0])
    except (KeyError, ValueError, IndexError):
        return default


def array(d, key):
    try:
        return [float(x) for x in d[key].strip("[]").split(",")]
    except (KeyError, ValueError):
        return []


class Stick:
    def __init__(self):
        self.x = self.y = self.r = self.t = 0
        self.busy = False
        self.land_asked = 0.0
        self.mode_requests = 0  # counts the operator's mode requests, so a takeoff never overrides a newer one
        self.message = "press t to take off"
        self.view = {"mode": "?", "speed": 0.0, "height": 0.0, "state": "?", "near": "-", "hook": ""}
        self.running = True

    def center(self):
        self.x = self.y = self.r = self.t = 0


def clamp(v):
    return max(-1000, min(1000, v))


def position(stick):
    px4("commander", "mode", "posctl")
    stick.message = "in Position: fly with the virtual joystick" if QGC else "in Position: fly with the keys"


def takeoff(stick):
    stick.busy = True
    stick.center()
    if -num(listen("vehicle_local_position"), "z", 0) > 1.0:  # already in the air: only switch to Position
        position(stick)
        stick.busy = False
        return
    stick.message = "arming and taking off..."
    requests = stick.mode_requests
    px4("commander", "arm", "-f")
    time.sleep(1)
    px4("commander", "takeoff")
    for _ in range(60):
        if -num(listen("vehicle_local_position"), "z", 0) > 3.0 or stick.mode_requests != requests:
            break
        time.sleep(0.5)
    if stick.mode_requests == requests:
        time.sleep(3)
    if stick.mode_requests == requests:  # Hold or Land asked meanwhile: keep it, never switch back to Position
        position(stick)
    stick.busy = False


def unlatch(stick):
    stick.busy = True
    stick.center()
    time.sleep(1)
    px4("param", "set", "CPB_EN", "0")
    time.sleep(2)
    px4("param", "set", "CPB_EN", "1")
    stick.message = "cp_brake switched off and on again"
    stick.busy = False


def press(stick, key):
    if QGC and key in " wsadqerf":
        stick.message = "the stick is the QGroundControl virtual joystick"
    elif key == " ":
        stick.center()
        stick.message = "stick centred"
    elif key in "ws":
        stick.x = clamp(stick.x + (STEP if key == "w" else -STEP))
    elif key in "ad":
        stick.y = clamp(stick.y + (STEP if key == "d" else -STEP))
    elif key in "qe":
        stick.r = clamp(stick.r + (200 if key == "e" else -200))
    elif key in "rf":
        stick.t = max(-400, min(400, stick.t + (200 if key == "r" else -200)))
    elif key == "t" and not stick.busy:
        threading.Thread(target=takeoff, args=(stick,), daemon=True).start()
    elif key == "p" and not stick.busy:
        stick.mode_requests += 1
        stick.center()
        threading.Thread(target=position, args=(stick,), daemon=True).start()
    elif key == "u" and not stick.busy:
        threading.Thread(target=unlatch, args=(stick,), daemon=True).start()
    elif key == "l":
        if time.time() - stick.land_asked < 3:
            stick.mode_requests += 1
            stick.center()
            threading.Thread(target=px4, args=("commander", "land"), daemon=True).start()
            stick.message = "landing"
        else:
            stick.land_asked = time.time()
            stick.message = "press l again to land (away from obstacles)"
    elif key == "h":
        stick.mode_requests += 1
        threading.Thread(target=px4, args=("commander", "mode", "auto:loiter"), daemon=True).start()
        stick.message = "Hold"
    elif key == "x":
        stick.running = False


def watch(stick):
    while stick.running:
        vs, lp = listen("vehicle_status"), listen("vehicle_local_position")
        cs, hk, od = listen("cp_brake_status"), listen("cp_brake_hook"), listen("obstacle_distance")
        view = {"mode": MODES.get(int(num(vs, "nav_state", -1)), "?"),
                "speed": math.hypot(num(lp, "vx", 0), num(lp, "vy", 0)), "height": -num(lp, "z", 0),
                "state": STATES.get(int(num(cs, "state", -1)), "?"), "near": "-", "hook": "no"}
        dist = array(od, "distances")
        if len(dist) == 72:
            seen = [(v, i) for i, v in enumerate(dist) if v < 2000]
            if seen:
                v, i = min(seen)
                view["near"] = "%.1f m at %+d deg from the nose" % (v / 100.0, i * 5 if i <= 36 else i * 5 - 360)
            else:
                view["near"] = "none within 20 m"
        action, sector = int(num(hk, "action", -1)), int(num(hk, "active_sector", 255))
        if view["mode"] not in ("Position", "MISSION"):
            view["hook"] = "cp_brake does not act in this mode"
        elif action == 3:
            out = array(hk, "vel_sp_out")
            why = "side speed lock" if sector == 255 else "obstacle at %+d deg" % (sector * 5 if sector <= 36 else sector * 5 - 360)
            view["hook"] = "YES (%s) -> %.1f m/s" % (why, math.hypot(*out) if len(out) == 2 else float("nan"))
        elif action == 4:
            view["hook"] = "SAFE STOP (press u away from obstacles)"
        if int(num(hk, "reason_flags", 0)) & 8192:
            view["hook"] += " [unprotected direction]"
        mc = listen("manual_control_setpoint")
        age = re.search(r"\(([\d.]+) seconds ago\)", mc.get("timestamp", ""))
        if age and float(age.group(1)) < 1.0 and mc.get("valid", "").startswith("True"):
            view["qgc"] = "fwd %+.2f  side %+.2f  yaw %+.2f  up %+.2f" % (
                num(mc, "pitch", 0), num(mc, "roll", 0), num(mc, "yaw", 0), num(mc, "throttle", 0))
        else:
            view["qgc"] = "NONE RECEIVED: open the QGroundControl Fly view"
        stick.view = view
        if view["height"] > 1.0 and view["mode"] == "Position" and stick.message == "press t to take off":
            stick.message = "flying in Position: use the virtual joystick" if QGC else "flying in Position"
        if view["height"] > 1.0 and view["mode"] == "Hold" and not stick.busy:
            stick.message = "in Hold: press p for Position, then fly"
        time.sleep(0.2)


def block(stick):
    v = stick.view
    return [
        "MODE     %-9s %4.1f m/s   height %4.1f m" % (v["mode"], v["speed"], v["height"]),
        "CP       %s" % v["state"],
        "NEAREST  %s" % v["near"],
        "LIMITED  %s" % v["hook"],
        "QGC      %s" % v.get("qgc", "-") if QGC else
        "STICK    fwd %+5d  side %+5d  yaw %+4d  up %+4d" % (stick.x, stick.y, stick.r, stick.t),
        "> %s" % stick.message,
    ]


def main():
    script = [(float(t), k.replace("_", " ")) for t, k in
              (item.split(":") for item in os.environ.get("KEYS", "").split(",") if item)]
    link = mavutil.mavlink_connection("udpin:0.0.0.0:14540")  # the onboard link; QGroundControl holds 14550
    if link.wait_heartbeat(timeout=30) is None:
        print("no PX4 heartbeat on port 14540")
        return 1
    stick = Stick()
    threading.Thread(target=watch, args=(stick,), daemon=True).start()
    print(HELP_QGC if QGC else HELP)
    old = None
    if not script and sys.stdin.isatty():
        old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
    start, last_print, drawn = time.time(), 0.0, 0
    try:
        while stick.running:
            if script and time.time() - start >= script[0][0]:
                press(stick, script.pop(0)[1])
            if old is not None and select.select([sys.stdin], [], [], 0)[0]:
                for key in os.read(sys.stdin.fileno(), 32).decode(errors="ignore").lower():
                    press(stick, key)
            # the stream is never read after the heartbeat: draining it made the loop too slow for PX4 to accept the stick
            if not QGC:
                link.mav.manual_control_send(link.target_system, stick.x, stick.y, 500 + stick.t, stick.r, 0)
            if time.time() - last_print > (1.0 if old is None else 0.25):
                last_print = time.time()
                if old is None:
                    print("%6.1f %s" % (time.time() - start, " | ".join(block(stick))), flush=True)
                else:  # redraw the same few lines in place, never wider than the terminal
                    width = shutil.get_terminal_size().columns - 1
                    lines = [line[:width] for line in block(stick)]
                    sys.stdout.write(("\033[%dA" % drawn if drawn else "") + "".join("\r\033[K" + s + "\n" for s in lines))
                    sys.stdout.flush()
                    drawn = len(lines)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if old is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
