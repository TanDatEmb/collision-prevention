#!/usr/bin/env python3
"""Live ROS 2 and PX4 telemetry bridge for the collision-prevention dashboard.

The bridge deliberately keeps the browser contract small. ROS owns the obstacle map and vehicle
state. PX4's collision-prevention messages can be sampled with ``px4-listener`` while the DDS
export is being added, so a running SITL remains observable during the transition. No
browser-side calculation is used for safety decisions.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import queue
import re
import subprocess
import threading
import time
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Mapping
from urllib.parse import urlsplit


_FIELD_RE = re.compile(r"^\s*([A-Za-z_]\w*(?:\[\d+\])?)\s*:\s*(.*?)\s*$")
_INDEX_RE = re.compile(r"^(?P<name>[A-Za-z_]\w*)\[(?P<index>\d+)\]$")
_NAV_STATES = {
    0: "Manual",
    1: "AltCtl",
    2: "Position",
    3: "Mission",
    4: "Hold",
    5: "RTL",
    10: "Acro",
    12: "Descend",
    13: "Terminate",
    14: "Offboard",
    15: "Stabilized",
    17: "Takeoff",
    18: "Land",
    20: "Precision land",
    21: "Orbit",
}
_STATE_NAMES = {
    0: "Disabled",
    1: "Observer",
    2: "Not ready",
    3: "Clear",
    4: "Braking",
    5: "Safe stop",
}
_ACTION_NAMES = {
    0: "Transparent",
    1: "No action",
    2: "Clear",
    3: "Limited",
    4: "Safe stop",
}


def _parse_token(value: str) -> Any:
    token = value.strip()
    if " (" in token:
        token = token.split(" (", 1)[0].strip()
    lower = token.lower()
    if lower == "true":
        return True
    if lower == "false":
        return False
    if lower in {"nan", "+nan", "-nan"}:
        return "nan"
    if lower in {"inf", "+inf", "infinity", "+infinity"}:
        return "inf"
    if lower in {"-inf", "-infinity"}:
        return "-inf"
    try:
        if re.fullmatch(r"[+-]?\d+", token):
            return int(token)
        return float(token)
    except ValueError:
        return token


def _parse_value(value: str) -> Any:
    value = value.strip()
    if value.startswith("[") and value.endswith("]"):
        inner = value[1:-1].strip()
        return [] if not inner else [_parse_token(item) for item in inner.split(",")]
    return _parse_token(value)


def parse_listener_output(output: str) -> dict[str, Any]:
    """Parse the stable ``px4-listener`` field format.

    PX4 prints fixed arrays either as one bracketed value or as indexed fields, depending on the
    firmware build. Indexed values are kept as a sparse dictionary so a missing field cannot be
    silently rendered as a valid zero.
    """

    result: dict[str, Any] = {}
    for line in output.splitlines():
        match = _FIELD_RE.match(line)
        if not match:
            continue
        key, raw_value = match.groups()
        value = _parse_value(raw_value)
        index = _INDEX_RE.match(key)
        if index:
            result.setdefault(index.group("name"), {})[int(index.group("index"))] = value
        else:
            result[key] = value
    return result


def _number(value: Any, default: float | None = None) -> float | None:
    if isinstance(value, (bytes, bytearray)):
        try:
            return float(int.from_bytes(value, "little"))
        except (TypeError, ValueError):
            return default
    if isinstance(value, bool):
        return float(value)
    if isinstance(value, (int, float)):
        number = float(value)
        return number if math.isfinite(number) else default
    try:
        number = float(str(value))
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _integer(value: Any, default: int | None = None) -> int | None:
    number = _number(value)
    return default if number is None else int(number)


def _list_value(value: Any, length: int | None = None) -> list[Any]:
    if isinstance(value, Mapping):
        if not value:
            result: list[Any] = []
        else:
            result = [None] * (max(int(index) for index in value) + 1)
            for index, item in value.items():
                result[int(index)] = item
    elif isinstance(value, (list, tuple)):
        result = list(value)
    else:
        result = []
    if length is not None:
        result = (result + [None] * length)[:length]
    return result


def _quaternion_attitude(q: list[Any]) -> tuple[float | None, float | None]:
    values = [_number(item) for item in _list_value(q, 4)]
    if any(item is None for item in values):
        return None, None
    w, x, y, z = values  # type: ignore[misc]
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-9:
        return None, None
    w, x, y, z = (item / norm for item in (w, x, y, z))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_arg = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_arg)
    return math.degrees(roll), math.degrees(pitch)


def _rotation_x(angle: float) -> tuple[tuple[float, float, float], ...]:
    cosine, sine = math.cos(angle), math.sin(angle)
    return ((1.0, 0.0, 0.0), (0.0, cosine, -sine), (0.0, sine, cosine))


def _rotation_y(angle: float) -> tuple[tuple[float, float, float], ...]:
    cosine, sine = math.cos(angle), math.sin(angle)
    return ((cosine, 0.0, sine), (0.0, 1.0, 0.0), (-sine, 0.0, cosine))


def _rotation_z(angle: float) -> tuple[tuple[float, float, float], ...]:
    cosine, sine = math.cos(angle), math.sin(angle)
    return ((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0))


def _mat_vec(matrix: tuple[tuple[float, float, float], ...], vector: tuple[float, float, float]):
    return tuple(sum(row[index] * vector[index] for index in range(3)) for row in matrix)


def _mat_mul(
    left: tuple[tuple[float, float, float], ...], right: tuple[tuple[float, float, float], ...]
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        tuple(sum(left[row][index] * right[index][column] for index in range(3)) for column in range(3))
        for row in range(3)
    )


def _view_elevation_deg(
    sensor_elevation_rad: float,
    azimuth_rad: float,
    mount_roll_rad: float,
    mount_pitch_down_rad: float,
    mount_yaw_left_rad: float,
    aircraft_roll_rad: float,
    aircraft_pitch_rad: float,
) -> float:
    """Transform one sensor-frame ray to the gravity-aligned frame used by the map."""

    cos_el = math.cos(sensor_elevation_rad)
    sensor_ray = (
        cos_el * math.cos(azimuth_rad),
        cos_el * math.sin(azimuth_rad),
        math.sin(sensor_elevation_rad),
    )
    mount = _mat_mul(
        _rotation_z(mount_yaw_left_rad),
        _mat_mul(_rotation_y(mount_pitch_down_rad), _rotation_x(mount_roll_rad)),
    )
    # Positive aircraft pitch is nose-up. In FLU, a positive mount-style Y rotation points down, so
    # the body-to-level pitch is the inverse; positive roll keeps the left side positive-up.
    body_ray = _mat_vec(mount, sensor_ray)
    level = _mat_mul(_rotation_x(aircraft_roll_rad), _rotation_y(-aircraft_pitch_rad))
    level_ray = _mat_vec(level, body_ray)
    return math.degrees(math.atan2(level_ray[2], math.hypot(level_ray[0], level_ray[1])))


def _view_ranges(geometry: Mapping[str, Any], pitch_deg: float, roll_deg: float) -> list[dict[str, float]]:
    mount_roll = math.radians(float(geometry["mount_roll_deg"]))
    mount_pitch = math.radians(float(geometry["mount_pitch_down_deg"]))
    mount_yaw = math.radians(float(geometry["mount_yaw_left_deg"]))
    sensor_min = math.radians(float(geometry["sensor_min_deg"]))
    sensor_max = math.radians(float(geometry["sensor_max_deg"]))
    aircraft_pitch = math.radians(pitch_deg)
    aircraft_roll = math.radians(roll_deg)
    result = []
    for index in range(72):
        bearing_deg = index * 5.0
        azimuth = math.radians(bearing_deg)
        edges = (
            _view_elevation_deg(
                sensor_min,
                azimuth,
                mount_roll,
                mount_pitch,
                mount_yaw,
                aircraft_roll,
                aircraft_pitch,
            ),
            _view_elevation_deg(
                sensor_max,
                azimuth,
                mount_roll,
                mount_pitch,
                mount_yaw,
                aircraft_roll,
                aircraft_pitch,
            ),
        )
        result.append({"bearing_deg": bearing_deg, "min_deg": min(edges), "max_deg": max(edges)})
    return result


def _copy_or_none(value: Any) -> Any:
    return None if value is None else copy.deepcopy(value)


def sse_event(event: str, payload: Any) -> str:
    """Encode one server-sent event without allowing line breaks in the JSON payload."""

    data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
    return f"event: {event}\ndata: {data}\n\n"


class SnapshotStore:
    """Thread-safe latest-value store shared by ROS, PX4 polling and HTTP clients."""

    def __init__(
        self,
        clock: Callable[[], float] = time.monotonic,
        source_timeout_s: float = 2.0,
        event_interval_s: float = 1.0 / 30.0,
    ):
        self._clock = clock
        self._source_timeout_s = float(source_timeout_s)
        self._event_interval_s = max(0.0, float(event_interval_s))
        self._last_event_at: float | None = None
        self._lock = threading.RLock()
        self._subscribers: set[queue.Queue[dict[str, str]]] = set()
        self._received: dict[str, float] = {}
        self._errors: dict[str, str] = {}
        self._map: dict[str, Any] | None = None
        self._caps: dict[str, Any] | None = None
        self._hook: dict[str, Any] | None = None
        self._diagnostics: dict[str, Any] | None = None
        self._aircraft: dict[str, Any] = {
            "mode": "—",
            "armed": None,
            "speed_m_s": None,
            "height_m": None,
            "pitch_deg": None,
            "roll_deg": None,
            "timestamp_us": None,
        }
        self._leveling: dict[str, Any] = {
            "status": "no_ray_pairs",
            "reference_time_us": None,
            "pitch_deg": None,
            "roll_deg": None,
            "pairs": None,
            "geometry": None,
            "view": None,
            "view_by_bin": None,
            "band": None,
            "coverage": None,
        }

    def _touch(self, source: str) -> None:
        self._received[source] = self._clock()

    def _notify_locked(self) -> None:
        now = self._clock()
        if self._last_event_at is not None and now - self._last_event_at < self._event_interval_s:
            return
        self._last_event_at = now
        event = {
            "event": "snapshot",
            "data": json.dumps(self._snapshot_locked(), ensure_ascii=False, separators=(",", ":")),
        }
        for subscriber in tuple(self._subscribers):
            try:
                subscriber.put_nowait(event)
            except queue.Full:
                try:
                    subscriber.get_nowait()
                except queue.Empty:
                    pass
                try:
                    subscriber.put_nowait(event)
                except queue.Full:
                    pass

    def _update_locked(self, source: str) -> None:
        self._touch(source)
        self._notify_locked()

    def update_map(self, payload: Mapping[str, Any]) -> None:
        distances = [
            _integer(item)
            for item in _list_value(payload.get("distances", payload.get("distances_cm")), 72)
        ]
        with self._lock:
            self._map = {
                "timestamp_us": _integer(payload.get("timestamp"), 0),
                "frame": _integer(payload.get("frame")),
                "sensor_type": _integer(payload.get("sensor_type")),
                "increment_deg": _number(payload.get("increment", payload.get("increment_deg"))),
                "angle_offset_deg": _number(payload.get("angle_offset", payload.get("angle_offset_deg"))),
                "min_distance_cm": _integer(payload.get("min_distance", payload.get("min_distance_cm"))),
                "max_distance_cm": _integer(payload.get("max_distance", payload.get("max_distance_cm"))),
                "distances_cm": distances,
            }
            self._update_locked("map")

    def update_odometry(self, payload: Mapping[str, Any]) -> None:
        position = [_number(item) for item in _list_value(payload.get("position"), 3)]
        velocity = [_number(item) for item in _list_value(payload.get("velocity"), 3)]
        roll, pitch = _quaternion_attitude(_list_value(payload.get("q"), 4))
        height = None if position[2] is None else -position[2]
        speed = None
        if velocity[0] is not None and velocity[1] is not None:
            speed = math.hypot(velocity[0], velocity[1])
        with self._lock:
            self._aircraft.update(
                {
                    "speed_m_s": speed,
                    "height_m": height,
                    "pitch_deg": pitch,
                    "roll_deg": roll,
                    "timestamp_us": _integer(payload.get("timestamp"), 0),
                }
            )
            self._leveling.update(
                {
                    "reference_time_us": _integer(
                        payload.get("timestamp_sample", payload.get("timestamp")), 0
                    ),
                    "pitch_deg": pitch,
                    "roll_deg": roll,
                }
            )
            self._recompute_leveling_view_locked()
            if self._leveling["status"] == "no_ray_pairs":
                self._leveling["status"] = "pose_only"
            self._update_locked("odometry")

    def update_status(self, payload: Mapping[str, Any]) -> None:
        nav_state = _integer(payload.get("nav_state"), -1)
        arming_state = _integer(payload.get("arming_state"), -1)
        with self._lock:
            self._aircraft.update(
                {
                    "mode": _NAV_STATES.get(nav_state, "Unknown"),
                    "armed": arming_state == 2 if arming_state is not None else None,
                }
            )
            self._update_locked("status")

    def update_caps(self, payload: Mapping[str, Any]) -> None:
        caps = [_number(item) for item in _list_value(payload.get("caps_m_s"), 72)]
        distances = [_number(item) for item in _list_value(payload.get("distances_m"), 72)]
        state = _integer(payload.get("state"))
        with self._lock:
            self._caps = {
                "timestamp_us": _integer(payload.get("timestamp"), 0),
                "sample_timestamp_us": _integer(payload.get("sample_timestamp"), 0),
                "heading_rad": _number(payload.get("heading_rad")),
                "heading_valid": bool(payload.get("heading_valid")) if "heading_valid" in payload else None,
                "caps_m_s": caps,
                "distances_m": distances,
                "clearance_m": _number(payload.get("clearance_m")),
                "enforcement_active": bool(payload.get("enforcement_active"))
                if "enforcement_active" in payload
                else None,
                "valid": bool(payload.get("valid")) if "valid" in payload else None,
                "state": state,
                "state_name": _STATE_NAMES.get(state, "Unknown"),
                "reason_flags": _integer(payload.get("reason_flags"), 0),
            }
            self._update_locked("caps")

    def update_hook(self, payload: Mapping[str, Any]) -> None:
        action = _integer(payload.get("action"))
        with self._lock:
            self._hook = {
                "timestamp_us": _integer(payload.get("timestamp"), 0),
                "sample_timestamp_us": _integer(payload.get("sample_timestamp"), 0),
                "constraint_timestamp_us": _integer(payload.get("constraint_timestamp"), 0),
                "sample_age_us": _integer(payload.get("sample_age_us")),
                "constraint_age_us": _integer(payload.get("constraint_age_us")),
                "state": _integer(payload.get("state")),
                "reason_flags": _integer(payload.get("reason_flags"), 0),
                "action": action,
                "action_name": _ACTION_NAMES.get(action, "Unknown"),
                "active_sector": _integer(payload.get("active_sector"), 255),
                "vel_sp_in_m_s": [_number(item) for item in _list_value(payload.get("vel_sp_in"), 2)],
                "vel_sp_out_m_s": [_number(item) for item in _list_value(payload.get("vel_sp_out"), 2)],
                "velocity_m_s": [_number(item) for item in _list_value(payload.get("vel"), 2)],
            }
            self._update_locked("hook")

    def update_diagnostics(self, payload: Mapping[str, Any]) -> None:
        with self._lock:
            self._diagnostics = _copy_or_none(dict(payload))
            self._update_locked("diagnostics")

    def _recompute_leveling_view_locked(self) -> None:
        geometry = self._leveling.get("geometry")
        pitch = _number(self._leveling.get("pitch_deg"))
        roll = _number(self._leveling.get("roll_deg"))
        if not isinstance(geometry, Mapping) or pitch is None or roll is None:
            self._leveling["view"] = None
            self._leveling["view_by_bin"] = None
            return
        views = _view_ranges(geometry, pitch, roll)
        self._leveling["view_by_bin"] = views
        self._leveling["view"] = views[0]

    def update_leveling_coverage(self, payload: Mapping[str, Any]) -> None:
        """Store the node's per-bin vertical coverage without treating it as a safety decision."""

        required = max(0, _integer(payload.get("required_rays"), 0) or 0)
        up = [max(0, _integer(item, 0) or 0) for item in _list_value(payload.get("up_ray_count"), 72)]
        down = [max(0, _integer(item, 0) or 0) for item in _list_value(payload.get("down_ray_count"), 72)]
        rear = [bool(item) for item in _list_value(payload.get("rear_only"), 72)]
        has_supplied_ok = "coverage_ok" in payload
        supplied_ok = _list_value(payload.get("coverage_ok"), 72) if has_supplied_ok else []
        packet_continuity = bool(payload.get("packet_continuity_valid"))
        geometry = {
            "mount_roll_deg": math.degrees(_number(payload.get("mount_roll_rad"), 0.0) or 0.0),
            "mount_pitch_down_deg": math.degrees(
                _number(payload.get("mount_pitch_down_rad"), 0.0) or 0.0
            ),
            "mount_yaw_left_deg": math.degrees(_number(payload.get("mount_yaw_left_rad"), 0.0) or 0.0),
            "sensor_min_deg": math.degrees(
                _number(payload.get("sensor_elevation_min_rad"), 0.0) or 0.0
            ),
            "sensor_max_deg": math.degrees(
                _number(payload.get("sensor_elevation_max_rad"), 0.0) or 0.0
            ),
        }
        band = {
            "height_up_m": _number(payload.get("height_up_m")),
            "height_down_m": _number(payload.get("height_down_m")),
            "attitude_allowance_deg": math.degrees(
                _number(payload.get("attitude_allowance_rad"), 0.0) or 0.0
            ),
            "required_rays": required,
        }
        bins = []
        for index in range(72):
            computed_ok = up[index] >= required and (rear[index] or down[index] >= required)
            coverage_ok = bool(supplied_ok[index]) if has_supplied_ok else computed_ok
            bins.append(
                {
                    "up_ray_count": up[index],
                    "down_ray_count": down[index],
                    "rear_only": rear[index],
                    "coverage_ok": coverage_ok,
                    "state": "Covered" if packet_continuity and coverage_ok else "Unknown",
                    "reason": "Covered"
                    if packet_continuity and coverage_ok
                    else ("Packet gap" if not packet_continuity else "Coverage gap"),
                }
            )
        with self._lock:
            self._leveling.update(
                {
                    "status": "coverage_live"
                    if _number(self._leveling.get("pitch_deg")) is not None
                    else "coverage_waiting",
                    "reference_time_us": _integer(payload.get("sample_timestamp"), 0),
                    "geometry": geometry,
                    "band": band,
                    "coverage": {
                        "timestamp_us": _integer(payload.get("timestamp"), 0),
                        "sample_timestamp_us": _integer(payload.get("sample_timestamp"), 0),
                        "packet_continuity_valid": packet_continuity,
                        "required_rays": required,
                        "bins": bins,
                    },
                }
            )
            self._recompute_leveling_view_locked()
            self._update_locked("leveling")

    def update_leveling_pairs(self, payload: Mapping[str, Any]) -> None:
        """Accept an optional future debug stream without fabricating pairs from the map."""

        with self._lock:
            pairs = payload.get("pairs")
            has_pose = any(
                _number(self._leveling.get(name)) is not None for name in ("pitch_deg", "roll_deg")
            )
            self._leveling.update(
                {
                    "status": "live" if pairs else ("pose_only" if has_pose else "no_ray_pairs"),
                    "reference_time_us": _integer(payload.get("reference_time_us")),
                    "pitch_deg": _number(payload.get("pitch_deg")),
                    "roll_deg": _number(payload.get("roll_deg")),
                    "pairs": _copy_or_none(pairs),
                }
            )
            self._update_locked("leveling")

    def set_error(self, source: str, message: str) -> None:
        with self._lock:
            self._errors[source] = message
            self._notify_locked()

    def subscribe(self) -> queue.Queue[dict[str, str]]:
        subscriber: queue.Queue[dict[str, str]] = queue.Queue(maxsize=2)
        with self._lock:
            self._subscribers.add(subscriber)
        return subscriber

    def unsubscribe(self, subscriber: queue.Queue[dict[str, str]]) -> None:
        with self._lock:
            self._subscribers.discard(subscriber)

    def _snapshot_locked(self) -> dict[str, Any]:
        now = self._clock()
        sources: dict[str, Any] = {}
        for source in ("map", "odometry", "status", "caps", "hook", "diagnostics"):
            received = self._received.get(source)
            age = None if received is None else max(0.0, now - received)
            sources[source] = {
                "state": "missing" if age is None else ("live" if age <= self._source_timeout_s else "stale"),
                "age_s": age,
            }
        seen = [entry["age_s"] is not None for entry in sources.values()]
        fresh_ros = any(sources[name]["state"] == "live" for name in ("map", "odometry", "status"))
        map_and_odo = all(sources[name]["state"] == "live" for name in ("map", "odometry"))
        if not any(seen):
            connection = "waiting"
        elif map_and_odo:
            connection = "connected"
        elif fresh_ros or any(entry["state"] == "live" for entry in sources.values()):
            connection = "partial"
        else:
            connection = "stale"
        timestamp_us = None
        for value in (self._map, self._hook, self._caps):
            if value and value.get("timestamp_us"):
                timestamp_us = value["timestamp_us"]
                break
        return {
            "connection": connection,
            "server_time_s": now,
            "timestamp_us": timestamp_us,
            "sources": sources,
            "errors": dict(self._errors),
            "aircraft": copy.deepcopy(self._aircraft),
            "map": copy.deepcopy(self._map),
            "caps": copy.deepcopy(self._caps),
            "hook": copy.deepcopy(self._hook),
            "diagnostics": copy.deepcopy(self._diagnostics),
            "leveling": copy.deepcopy(self._leveling),
        }

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._snapshot_locked()


class Px4Listener:
    """Small subprocess adapter for PX4's local uORB listener command."""

    def __init__(self, binary: str | os.PathLike[str] | None = None, runner: Callable[..., Any] = subprocess.run):
        configured = binary or os.environ.get("PX4_LISTENER")
        if configured:
            self.binary = Path(configured)
        else:
            build = Path(os.environ.get("PX4_BUILD", "firmware/PX4-Autopilot/build/px4_sitl_default"))
            if not build.is_absolute():
                build = Path(__file__).resolve().parents[2] / build
            self.binary = build / "bin" / "px4-listener"
        self._runner = runner

    def poll(self, topic: str) -> dict[str, Any] | None:
        if not self.binary.exists():
            return None
        try:
            result = self._runner(
                [str(self.binary), topic, "-n", "1"],
                capture_output=True,
                text=True,
                timeout=0.8,
                check=False,
            )
        except (OSError, subprocess.SubprocessError):
            return None
        if getattr(result, "returncode", 1) != 0:
            return None
        parsed = parse_listener_output(getattr(result, "stdout", ""))
        # px4-listener exits with status 0 for a topic-size error and still prints a ``TOPIC``
        # header. Do not turn that metadata into a zero-filled dashboard sample.
        return parsed if "timestamp" in parsed else None


class ListenerPoller(threading.Thread):
    def __init__(self, store: SnapshotStore, listener: Px4Listener, stop_event: threading.Event, interval_s: float = 0.2):
        super().__init__(name="px4-listener", daemon=True)
        self._store = store
        self._listener = listener
        self._stop_event = stop_event
        self._interval_s = interval_s

    def run(self) -> None:
        while not self._stop_event.is_set():
            caps = self._listener.poll("cp_brake_constraints")
            if caps:
                self._store.update_caps(caps)
            hook = self._listener.poll("cp_brake_hook")
            if hook:
                self._store.update_hook(hook)
            self._stop_event.wait(self._interval_s)


class RosTelemetryThread(threading.Thread):
    """Spin ROS subscriptions without importing rclpy when unit-testing the pure bridge."""

    def __init__(self, store: SnapshotStore, stop_event: threading.Event):
        super().__init__(name="ros-telemetry", daemon=True)
        self._store = store
        self._stop_event = stop_event
        self._rclpy = None

    def run(self) -> None:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from diagnostic_msgs.msg import DiagnosticArray
            from px4_msgs.msg import ObstacleDistance, VehicleOdometry, VehicleStatus

            try:
                from collision_prevention_msgs.msg import CpBrakeConstraints, CpBrakeHook, CpLevelingCoverage
            except ImportError as exc:
                CpBrakeConstraints = None
                CpBrakeHook = None
                CpLevelingCoverage = None
                self._store.set_error("dds", f"monitor messages unavailable: {exc}")

            self._rclpy = rclpy
            rclpy.init(args=None)
            node = rclpy.create_node("collision_prevention_dashboard")
            node.create_subscription(
                ObstacleDistance,
                "/fmu/in/obstacle_distance",
                lambda message: self._store.update_map(
                    {
                        "timestamp": message.timestamp,
                        "frame": message.frame,
                        "sensor_type": message.sensor_type,
                        "distances": list(message.distances),
                        "increment": message.increment,
                        "min_distance": message.min_distance,
                        "max_distance": message.max_distance,
                        "angle_offset": message.angle_offset,
                    }
                ),
                qos_profile_sensor_data,
            )
            node.create_subscription(
                VehicleOdometry,
                "/fmu/out/vehicle_odometry",
                lambda message: self._store.update_odometry(
                    {
                        "timestamp": message.timestamp,
                        "timestamp_sample": message.timestamp_sample,
                        "position": list(message.position),
                        "velocity": list(message.velocity),
                        "q": list(message.q),
                    }
                ),
                qos_profile_sensor_data,
            )
            for topic in ("/fmu/out/vehicle_status_v1", "/fmu/out/vehicle_status"):
                node.create_subscription(
                    VehicleStatus,
                    topic,
                    lambda message: self._store.update_status(
                        {
                            "nav_state": message.nav_state,
                            "arming_state": message.arming_state,
                        }
                    ),
                    qos_profile_sensor_data,
                )
            node.create_subscription(
                DiagnosticArray,
                "/status",
                lambda message: self._store.update_diagnostics(_diagnostics_payload(message)),
                10,
            )
            if CpBrakeConstraints is not None:
                node.create_subscription(
                    CpBrakeConstraints,
                    "/fmu/out/cp_brake_constraints",
                    lambda message: self._store.update_caps(
                        {
                            "timestamp": message.timestamp,
                            "sample_timestamp": message.sample_timestamp,
                            "heading_rad": message.heading_rad,
                            "heading_valid": message.heading_valid,
                            "caps_m_s": list(message.caps_m_s),
                            "distances_m": list(message.distances_m),
                            "clearance_m": message.clearance_m,
                            "enforcement_active": message.enforcement_active,
                            "valid": message.valid,
                            "state": message.state,
                            "reason_flags": message.reason_flags,
                        }
                    ),
                    qos_profile_sensor_data,
                )
            if CpBrakeHook is not None:
                node.create_subscription(
                    CpBrakeHook,
                    "/fmu/out/cp_brake_hook",
                    lambda message: self._store.update_hook(
                        {
                            "timestamp": message.timestamp,
                            "sample_timestamp": message.sample_timestamp,
                            "constraint_timestamp": message.constraint_timestamp,
                            "sample_age_us": message.sample_age_us,
                            "constraint_age_us": message.constraint_age_us,
                            "state": message.state,
                            "reason_flags": message.reason_flags,
                            "action": message.action,
                            "active_sector": message.active_sector,
                            "vel_sp_in": list(message.vel_sp_in),
                            "vel_sp_out": list(message.vel_sp_out),
                            "vel": list(message.vel),
                        }
                    ),
                    qos_profile_sensor_data,
                )
            if CpLevelingCoverage is not None:
                node.create_subscription(
                    CpLevelingCoverage,
                    "/collision_prevention/leveling_coverage",
                    lambda message: self._store.update_leveling_coverage(
                        {
                            "timestamp": message.timestamp,
                            "sample_timestamp": message.sample_timestamp,
                            "mount_roll_rad": message.mount_roll_rad,
                            "mount_pitch_down_rad": message.mount_pitch_down_rad,
                            "mount_yaw_left_rad": message.mount_yaw_left_rad,
                            "sensor_elevation_min_rad": message.sensor_elevation_min_rad,
                            "sensor_elevation_max_rad": message.sensor_elevation_max_rad,
                            "height_up_m": message.height_up_m,
                            "height_down_m": message.height_down_m,
                            "attitude_allowance_rad": message.attitude_allowance_rad,
                            "required_rays": message.required_rays,
                            "packet_continuity_valid": message.packet_continuity_valid,
                            "up_ray_count": list(message.up_ray_count),
                            "down_ray_count": list(message.down_ray_count),
                            "rear_only": list(message.rear_only),
                            "coverage_ok": list(message.coverage_ok),
                        }
                    ),
                    qos_profile_sensor_data,
                )
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            while not self._stop_event.is_set():
                executor.spin_once(timeout_sec=0.2)
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown()
        except Exception as exc:  # Keep the web server alive and expose the reason in the UI.
            self._store.set_error("ros", str(exc))
            if self._rclpy is not None and self._rclpy.ok():
                self._rclpy.shutdown()


def _diagnostics_payload(message: Any) -> dict[str, Any]:
    statuses = []
    for status in getattr(message, "status", []):
        values = {item.key: item.value for item in getattr(status, "values", [])}
        statuses.append(
            {
                "name": status.name,
                "level": _integer(status.level, 0),
                "message": status.message,
                "values": values,
            }
        )
    return {"statuses": statuses}


class DashboardHttpServer(ThreadingHTTPServer):
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], root: Path, store: SnapshotStore):
        super().__init__(address, DashboardRequestHandler)
        self.root = root
        self.store = store


class DashboardRequestHandler(BaseHTTPRequestHandler):
    server: DashboardHttpServer
    protocol_version = "HTTP/1.1"

    def log_message(self, _format: str, *_args: Any) -> None:
        return

    def _json(self, payload: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        path = urlsplit(self.path).path
        if path in ("/", "/index.html"):
            body = (self.server.root / "index.html").read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path in ("/snapshot", "/health"):
            self._json(self.server.store.snapshot())
            return
        if path == "/events":
            self._events()
            return
        self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def _events(self) -> None:
        subscriber = self.server.store.subscribe()
        try:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            self.wfile.write(sse_event("snapshot", self.server.store.snapshot()).encode("utf-8"))
            self.wfile.flush()
            while True:
                try:
                    event = subscriber.get(timeout=15.0)
                    encoded = f"event: {event['event']}\ndata: {event['data']}\n\n"
                except queue.Empty:
                    encoded = ": ping\n\n"
                self.wfile.write(encoded.encode("utf-8"))
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass
        finally:
            self.server.store.unsubscribe(subscriber)


def run_server(
    host: str,
    port: int,
    root: Path,
    px4_binary: str | None = None,
    open_browser: bool = False,
    enable_listener: bool = True,
    source_timeout_s: float = 2.0,
) -> None:
    store = SnapshotStore(source_timeout_s=source_timeout_s)
    stop_event = threading.Event()
    ros_thread = RosTelemetryThread(store, stop_event)
    ros_thread.start()
    listener_thread: ListenerPoller | None = None
    if enable_listener:
        listener_thread = ListenerPoller(store, Px4Listener(px4_binary), stop_event)
        listener_thread.start()
    httpd = DashboardHttpServer((host, port), root, store)
    print(f"CP Monitor: http://{host}:{port}", flush=True)
    if open_browser:
        threading.Timer(0.4, lambda: webbrowser.open(f"http://{host}:{port}/")).start()
    try:
        httpd.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        httpd.shutdown()
        httpd.server_close()
        ros_thread.join(timeout=2.0)
        if listener_thread is not None:
            listener_thread.join(timeout=2.0)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Live collision-prevention ROS/PX4 dashboard")
    parser.add_argument("--host", default=os.environ.get("DASHBOARD_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("DASHBOARD_PORT", "8765")))
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--px4-binary", default=os.environ.get("PX4_LISTENER"))
    parser.add_argument("--open", action="store_true", default=os.environ.get("DASHBOARD_OPEN") == "1")
    parser.add_argument("--no-listener", action="store_true")
    parser.add_argument("--source-timeout", type=float, default=2.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if not (args.root / "index.html").is_file():
        raise SystemExit(f"dashboard asset missing: {args.root / 'index.html'}")
    run_server(
        args.host,
        args.port,
        args.root,
        args.px4_binary,
        args.open,
        not args.no_listener,
        args.source_timeout,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
