#!/usr/bin/env python3
"""Report point-data statistics of a Livox LVX2 recording (OPEN-022 probe).

It answers factual questions only: are zero-range samples present, what do the tag bytes
contain, are packet counters and timestamps continuous, and how many rays fall in each of the
72 sectors per time window. It chooses no threshold and makes no safety claim.

LVX2 layout used (observed on the Livox Mid-360 sample files, not from a published spec; the
reader stops with an error when a file does not match it):
  public header 24 bytes: 16-byte signature "livox_tech", version (4 bytes), magic 0xac0ea767
  private header 5 bytes: frame duration in ms (u32), device count (u8)
  device info: 63 bytes per device
  frame: header 24 bytes (current offset u64, next offset u64, frame index u64), then packages
  package: 27-byte header (version u8, lidar id u32, lidar type u8, timestamp type u8,
           timestamp u64 ns, udp counter u16, data type u8, payload length u32, frame counter u8,
           4 reserved bytes) followed by the payload
  payload: data type 1 = int32 x, y, z in mm, reflectivity u8, tag u8; type 2 = int16 x, y, z in
           10 mm; type 3 = depth u32 mm, theta u16 and phi u16 in 0.01 deg, reflectivity, tag.
Statistics are kept per lidar id (own clock and packet counter). Needs Python 3 and numpy.

Usage: probe_lvx2.py FILE [--window-ms 100] [--json]    |    probe_lvx2.py --selftest
"""
import argparse
import json
import math
import mmap
import struct
import sys
import tempfile

import numpy as np

SIGNATURE = b"livox_tech" + bytes(6)
MAGIC = 0xAC0EA767
PUBLIC_HEADER = 24
PRIVATE_HEADER = 5
DEVICE_INFO = 63
FRAME_HEADER = struct.Struct("<QQQ")
PACKAGE_HEADER = struct.Struct("<BIBBQHBIB4x")
SECTORS = 72
POINT_DTYPES = {
    1: np.dtype([("x", "<i4"), ("y", "<i4"), ("z", "<i4"), ("r", "u1"), ("t", "u1")]),
    2: np.dtype([("x", "<i2"), ("y", "<i2"), ("z", "<i2"), ("r", "u1"), ("t", "u1")]),
    3: np.dtype([("d", "<u4"), ("theta", "<u2"), ("phi", "<u2"), ("r", "u1"), ("t", "u1")]),
}
FLUSH_PACKAGES = 4000


def fail(message):
    sys.exit("probe_lvx2: " + message)


def finite(value):
    return value if math.isfinite(value) else None


def decode(data_type, payload):
    """Return range_m, azimuth_rad, elevation_rad, zero_range, has_direction, reflectivity, tag.

    A Cartesian zero sample has no direction; a spherical one keeps theta and phi at zero depth.
    """
    pts = np.frombuffer(payload, dtype=POINT_DTYPES[data_type])
    if data_type == 3:
        rng = pts["d"].astype(np.float64) / 1000.0
        az = np.deg2rad(pts["phi"].astype(np.float64) * 0.01)
        el = np.deg2rad(90.0 - pts["theta"].astype(np.float64) * 0.01)
        zero = pts["d"] == 0
        has_dir = np.ones(len(zero), dtype=bool)
    else:
        scale = 1.0 if data_type == 1 else 10.0
        x = pts["x"].astype(np.float64) * scale
        y = pts["y"].astype(np.float64) * scale
        z = pts["z"].astype(np.float64) * scale
        rng = np.sqrt(x * x + y * y + z * z) / 1000.0
        az = np.arctan2(y, x)
        el = np.arctan2(z, np.hypot(x, y))
        zero = (pts["x"] == 0) & (pts["y"] == 0) & (pts["z"] == 0)
        has_dir = ~zero
    return rng, az, el, zero, has_dir, pts["r"], pts["t"]


class Stats:
    def __init__(self, window_ns):
        self.window_ns = window_ns
        self.points = 0
        self.zero_points = 0
        self.directional_zero_points = 0
        self.zero_reflectivity = 0
        self.tag_hist = np.zeros(256, dtype=np.int64)
        self.range_hist = np.zeros(1002, dtype=np.int64)  # 1 m bins, last bin = overflow
        self.range_min = math.inf
        self.range_max = -math.inf
        self.el_min = math.inf
        self.el_max = -math.inf
        self.windows = {}
        self.ts0 = None

    def add(self, data_type, payloads, stamps, counts):
        rng, az, el, zero, has_dir, refl, tag = decode(data_type, b"".join(payloads))
        ts = np.repeat(np.asarray(stamps, dtype=np.int64), counts)
        self.points += len(rng)
        self.zero_points += int(zero.sum())
        self.directional_zero_points += int((zero & has_dir).sum())
        self.zero_reflectivity += int((refl == 0).sum())
        self.tag_hist += np.bincount(tag, minlength=256)
        ok = ~zero
        if ok.any():
            self.range_min = min(self.range_min, float(rng[ok].min()))
            self.range_max = max(self.range_max, float(rng[ok].max()))
            self.el_min = min(self.el_min, float(el[ok].min()))
            self.el_max = max(self.el_max, float(el[ok].max()))
            hist = np.bincount(np.minimum(rng[ok], 1001).astype(np.int64), minlength=1002)
            self.range_hist += hist
        if not has_dir.any():
            return
        az, ts = az[has_dir], ts[has_dir]
        if self.ts0 is None:
            self.ts0 = int(ts.min())
        step = 2.0 * math.pi / SECTORS
        sector = np.floor((az + 0.5 * step) / step).astype(np.int64) % SECTORS
        window = (ts - self.ts0) // self.window_ns
        for w in np.unique(window):
            row = self.windows.setdefault(int(w), np.zeros(SECTORS, dtype=np.int64))
            row += np.bincount(sector[window == w], minlength=SECTORS)

    def range_percentile(self, q):
        cumulative = np.cumsum(self.range_hist)
        if cumulative[-1] == 0:
            return None
        return int(np.searchsorted(cumulative, q * cumulative[-1])) + 0.5


def window_summary(stats, window_ms):
    zero_row = np.zeros(SECTORS, dtype=np.int64)
    complete = []
    if stats.windows:
        span = range(min(stats.windows), max(stats.windows) + 1)
        complete = [stats.windows.get(w, zero_row) for w in span][1:-1]
    if not complete:
        return {"window_ms": window_ms, "windows": 0, "min": None, "p5": None, "median": None,
                "max": None, "empty_cells": None}
    cells = np.concatenate(complete)
    return {"window_ms": window_ms, "windows": len(complete), "min": int(cells.min()),
            "p5": int(np.percentile(cells, 5)), "median": int(np.median(cells)),
            "max": int(cells.max()), "empty_cells": int((cells == 0).sum())}


def read_file(path, window_ms):
    if not math.isfinite(window_ms) or int(window_ms * 1e6) < 1:
        fail("window must be at least 1 ns (got %r ms)" % window_ms)
    with open(path, "rb") as handle:
        buf = mmap.mmap(handle.fileno(), 0, access=mmap.ACCESS_READ)
    size = len(buf)
    if size < PUBLIC_HEADER + PRIVATE_HEADER or buf[:16] != SIGNATURE:
        fail("not an LVX2 file (signature)")
    version = list(buf[16:20])
    if struct.unpack_from("<I", buf, 20)[0] != MAGIC or version[0] != 2:
        fail("unsupported LVX version or magic code")
    frame_ms, devices = struct.unpack_from("<IB", buf, PUBLIC_HEADER)
    offset = PUBLIC_HEADER + PRIVATE_HEADER + devices * DEVICE_INFO
    report = {"file": path, "version": version, "frame_duration_ms": frame_ms, "devices": devices}
    frames = packages = imu_packages = 0
    data_types, time_types, lidar_types = set(), set(), set()
    stats = {}  # statistics are kept per lidar id: each lidar has its own clock and counters
    seen = {}
    per_frame = []
    pending = {}
    finished = False
    while offset + FRAME_HEADER.size <= size:
        current, nxt, _index = FRAME_HEADER.unpack_from(buf, offset)
        if current != offset:
            fail("frame header offset mismatch at %d" % offset)
        end = nxt if nxt else size
        if end > size or end <= offset:
            fail("bad next-frame offset at %d" % offset)
        pos, frame_points = offset + FRAME_HEADER.size, 0
        while pos + PACKAGE_HEADER.size <= end:
            fields = PACKAGE_HEADER.unpack_from(buf, pos)
            _v, lidar_id, ltype, ttype, ts, udp, dtype, length, _fc = fields
            body = pos + PACKAGE_HEADER.size
            if body + length > end:
                fail("package overruns its frame at %d" % pos)
            packages += 1
            data_types.add(dtype)
            time_types.add(ttype)
            lidar_types.add(ltype)
            if lidar_id not in stats:  # origin of this lidar's windows: its first package
                stats[lidar_id] = Stats(int(window_ms * 1e6))
                stats[lidar_id].ts0 = ts
            counters = seen.setdefault(lidar_id, ([], []))
            counters[0].append(udp)
            counters[1].append(ts)
            if dtype == 0:
                imu_packages += 1
            elif dtype not in POINT_DTYPES:
                fail("unsupported point data type %d at %d" % (dtype, pos))
            else:
                item = POINT_DTYPES[dtype].itemsize
                if length % item:
                    fail("payload length %d not a multiple of %d at %d" % (length, item, pos))
                key = (lidar_id, dtype)
                entry = pending.setdefault(key, ([], [], []))
                entry[0].append(bytes(buf[body:body + length]))
                entry[1].append(ts)
                entry[2].append(length // item)
                frame_points += length // item
                if len(entry[0]) >= FLUSH_PACKAGES:
                    stats[lidar_id].add(dtype, *pending.pop(key))
            pos = body + length
        if pos != end:
            fail("%d unconsumed bytes after the last package: truncated file" % (end - pos))
        per_frame.append(frame_points)
        frames += 1
        if not nxt:
            finished = True
            break
        offset = nxt
    # A last frame whose next-frame offset equals the file size is valid: the Livox Indoor
    # sample file ends that way. Any other early end means the file is cut.
    if not finished and offset != size:
        fail("truncated file: %d bytes remain after the last frame" % (size - offset))
    for (lidar_id, dtype), entry in pending.items():
        stats[lidar_id].add(dtype, *entry)
    report.update(frames=frames, packages=packages, imu_packages=imu_packages,
                  data_types=sorted(data_types), time_types=sorted(time_types),
                  lidar_types=sorted(lidar_types))
    per_lidar = {}
    for lidar_id, (counters, stamps) in sorted(seen.items()):
        deltas = np.diff(np.asarray(stamps, dtype=np.int64))
        gaps = int((np.diff(np.asarray(counters, dtype=np.int64)) % 65536 != 1).sum())
        st = stats.get(lidar_id)
        per_lidar[str(lidar_id)] = {
            "packages": len(counters), "udp_counter_gaps": gaps,
            "timestamp_delta_ns": {
                "min": int(deltas.min()) if len(deltas) else None,
                "median": int(np.median(deltas)) if len(deltas) else None,
                "max": int(deltas.max()) if len(deltas) else None,
                "backward_steps": int((deltas < 0).sum())},
            "points": st.points if st else 0,
            "zero_range_points": st.zero_points if st else 0,
            "rays_per_sector_per_window": window_summary(st, window_ms) if st else None}
    tag_hist = sum((st.tag_hist for st in stats.values()), np.zeros(256, dtype=np.int64))
    range_hist = sum((st.range_hist for st in stats.values()), np.zeros(1002, dtype=np.int64))
    merged = Stats(1)
    merged.range_hist = range_hist
    el_min = min((st.el_min for st in stats.values()), default=math.inf)
    el_max = max((st.el_max for st in stats.values()), default=-math.inf)
    points = sum(st.points for st in stats.values())
    zero_points = sum(st.zero_points for st in stats.values())
    report.update(
        per_lidar=per_lidar,
        points=points, zero_range_points=zero_points,
        zero_range_points_with_direction=sum(st.directional_zero_points for st in stats.values()),
        zero_range_fraction=zero_points / points if points else None,
        zero_reflectivity_points=sum(st.zero_reflectivity for st in stats.values()),
        points_per_frame={"min": min(per_frame), "median": int(np.median(per_frame)),
                          "max": max(per_frame)},
        tag_histogram={str(t): int(c) for t, c in enumerate(tag_hist) if c},
        range_m={"min": finite(min((st.range_min for st in stats.values()), default=math.inf)),
                 "max": finite(max((st.range_max for st in stats.values()), default=-math.inf)),
                 "p50_approx": merged.range_percentile(0.5),
                 "p95_approx": merged.range_percentile(0.95)},
        elevation_deg={"min": finite(math.degrees(el_min)), "max": finite(math.degrees(el_max))})
    return report


def selftest():
    """Write a tiny synthetic LVX2 file with known content and check the report."""
    def package(ts, udp, points, dtype=1):
        payload = b"".join(struct.pack("<iiiBB", *p) for p in points)
        head = PACKAGE_HEADER.pack(0, 1, 9, 0, ts, udp, dtype, len(payload), 0)
        return head + payload

    pts = [(1000, 0, 0, 10, 0), (0, 1000, 0, 10, 0), (0, 0, 0, 0, 0), (-1000, 0, 0, 10, 4)]
    start = PUBLIC_HEADER + PRIVATE_HEADER + DEVICE_INFO
    frames, offset = [], start
    for index in range(3):
        first = package(1_000_000 * index, 2 * index, pts)
        second = package(1_000_000 * index + 500, 2 * index + 1, pts)
        body = first + second
        size = FRAME_HEADER.size + len(body)
        nxt = offset + size if index < 2 else 0
        frames.append(FRAME_HEADER.pack(offset, nxt, index) + body)
        offset += size
    blob = SIGNATURE + bytes([2, 0, 0, 0]) + struct.pack("<I", MAGIC) + struct.pack("<IB", 50, 1)
    blob += bytes(DEVICE_INFO) + b"".join(frames)
    with tempfile.NamedTemporaryFile(suffix=".lvx2") as handle:
        handle.write(blob)
        handle.flush()
        report = read_file(handle.name, 1.0)
    expected = {"frames": 3, "packages": 6, "points": 24, "zero_range_points": 6,
                "zero_reflectivity_points": 6}
    for key, value in expected.items():
        if report[key] != value:
            fail("selftest %s: got %r, expected %r" % (key, report[key], value))
    if report["per_lidar"]["1"]["udp_counter_gaps"] != 0:
        fail("selftest udp counter gaps")
    if report["per_lidar"]["1"]["timestamp_delta_ns"]["backward_steps"] != 0:
        fail("selftest timestamp steps")
    if report["tag_histogram"] != {"0": 18, "4": 6}:
        fail("selftest tag histogram %r" % report["tag_histogram"])
    if list(report["per_lidar"]) != ["1"] or report["per_lidar"]["1"]["points"] != 24:
        fail("selftest per-lidar points")
    if abs(report["range_m"]["max"] - 1.0) > 1e-9 or abs(report["elevation_deg"]["max"]) > 1e-9:
        fail("selftest range or elevation")
    def rejected(data):
        with tempfile.NamedTemporaryFile(suffix=".lvx2") as handle:
            handle.write(data)
            handle.flush()
            try:
                read_file(handle.name, 1.0)
            except SystemExit:
                return True
        return False

    cut = blob[:-(len(frames[-1]) - 10)]
    bad_type = bytearray(blob)
    data_type_offset = start + FRAME_HEADER.size + 1 + 4 + 1 + 1 + 8 + 2  # package 0
    bad_type[data_type_offset] = 7
    spherical = Stats(1_000_000)
    zero_depth = struct.pack("<IHHBB", 0, 9000, 0, 0, 0)  # depth 0, theta 90 deg, phi 0
    spherical.add(3, [zero_depth], [0], [1])
    if spherical.zero_points != 1 or spherical.directional_zero_points != 1:
        fail("selftest: spherical zero-depth sample not counted")
    if sum(int(row.sum()) for row in spherical.windows.values()) != 1:
        fail("selftest: spherical zero-depth sample missing from the sector tally")
    for bad_window in (0.0, 1e-9, float("nan")):
        try:
            read_file("unused", bad_window)
        except SystemExit:
            continue
        fail("selftest: window %r was accepted" % bad_window)
    cases = {"trailing bytes": blob + bytes(5), "cut frame header": cut,
             "unknown data type": bytes(bad_type)}
    for name, data in cases.items():
        if not rejected(data):
            fail("selftest: %s was accepted" % name)
    print("selftest ok")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("file", nargs="?")
    parser.add_argument("--window-ms", type=float, default=100.0)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    if args.selftest:
        selftest()
        return
    if not args.file:
        parser.error("FILE is required")
    report = read_file(args.file, args.window_ms)
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        for key, value in report.items():
            print("%s: %s" % (key, value))


if __name__ == "__main__":
    main()
