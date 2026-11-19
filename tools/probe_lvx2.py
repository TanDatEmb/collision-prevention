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
Needs Python 3 and numpy.

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
    """Return range_m, azimuth_rad, elevation_rad, zero_range flag, reflectivity, tag."""
    pts = np.frombuffer(payload, dtype=POINT_DTYPES[data_type])
    if data_type == 3:
        rng = pts["d"].astype(np.float64) / 1000.0
        az = np.deg2rad(pts["phi"].astype(np.float64) * 0.01)
        el = np.deg2rad(90.0 - pts["theta"].astype(np.float64) * 0.01)
        zero = pts["d"] == 0
    else:
        scale = 1.0 if data_type == 1 else 10.0
        x = pts["x"].astype(np.float64) * scale
        y = pts["y"].astype(np.float64) * scale
        z = pts["z"].astype(np.float64) * scale
        rng = np.sqrt(x * x + y * y + z * z) / 1000.0
        az = np.arctan2(y, x)
        el = np.arctan2(z, np.hypot(x, y))
        zero = (pts["x"] == 0) & (pts["y"] == 0) & (pts["z"] == 0)
    return rng, az, el, zero, pts["r"], pts["t"]


class Stats:
    def __init__(self, window_ns):
        self.window_ns = window_ns
        self.points = 0
        self.zero_points = 0
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
        rng, az, el, zero, refl, tag = decode(data_type, b"".join(payloads))
        ts = np.repeat(np.asarray(stamps, dtype=np.int64), counts)
        self.points += len(rng)
        self.zero_points += int(zero.sum())
        self.zero_reflectivity += int((refl == 0).sum())
        self.tag_hist += np.bincount(tag, minlength=256)
        ok = ~zero
        if not ok.any():
            return
        rng, az, el, ts = rng[ok], az[ok], el[ok], ts[ok]
        self.range_min = min(self.range_min, float(rng.min()))
        self.range_max = max(self.range_max, float(rng.max()))
        self.el_min = min(self.el_min, float(el.min()))
        self.el_max = max(self.el_max, float(el.max()))
        self.range_hist += np.bincount(np.minimum(rng, 1001).astype(np.int64), minlength=1002)
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


def read_file(path, window_ms):
    stats = Stats(int(window_ms * 1e6))
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
    frames = packages = 0
    data_types, time_types, lidar_types = set(), set(), set()
    per_lidar, per_frame = {}, []
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
            seen = per_lidar.setdefault(lidar_id, ([], []))
            seen[0].append(udp)
            seen[1].append(ts)
            if dtype in POINT_DTYPES:
                item = POINT_DTYPES[dtype].itemsize
                if length % item:
                    fail("payload length %d not a multiple of %d at %d" % (length, item, pos))
                entry = pending.setdefault(dtype, ([], [], []))
                entry[0].append(bytes(buf[body:body + length]))
                entry[1].append(ts)
                entry[2].append(length // item)
                frame_points += length // item
                if len(entry[0]) >= FLUSH_PACKAGES:
                    stats.add(dtype, *pending.pop(dtype))
            pos = body + length
        if pos != end:
            fail("%d unconsumed bytes after the last package: truncated file" % (end - pos))
        per_frame.append(frame_points)
        frames += 1
        if not nxt:
            finished = True
            break
        offset = nxt
    if not finished and offset != size:
        fail("truncated file: %d bytes remain after the last frame" % (size - offset))
    for dtype, entry in pending.items():
        stats.add(dtype, *entry)
    report.update(frames=frames, packages=packages, data_types=sorted(data_types),
                  time_types=sorted(time_types), lidar_types=sorted(lidar_types))
    packets = {}
    for lidar_id, (counters, stamps) in sorted(per_lidar.items()):
        deltas = np.diff(np.asarray(stamps, dtype=np.int64))
        gaps = int((np.diff(np.asarray(counters, dtype=np.int64)) % 65536 != 1).sum())
        packets[str(lidar_id)] = {
            "packages": len(counters), "udp_counter_gaps": gaps,
            "timestamp_delta_ns": {
                "min": int(deltas.min()) if len(deltas) else None,
                "median": int(np.median(deltas)) if len(deltas) else None,
                "max": int(deltas.max()) if len(deltas) else None,
                "backward_steps": int((deltas < 0).sum())}}
    report.update(
        per_lidar=packets,
        points=stats.points, zero_range_points=stats.zero_points,
        zero_range_fraction=stats.zero_points / stats.points if stats.points else None,
        zero_reflectivity_points=stats.zero_reflectivity,
        points_per_frame={"min": min(per_frame), "median": int(np.median(per_frame)),
                          "max": max(per_frame)},
        tag_histogram={str(t): int(c) for t, c in enumerate(stats.tag_hist) if c},
        range_m={"min": finite(stats.range_min), "max": finite(stats.range_max),
                 "p50_approx": stats.range_percentile(0.5),
                 "p95_approx": stats.range_percentile(0.95)},
        elevation_deg={"min": finite(math.degrees(stats.el_min)),
                       "max": finite(math.degrees(stats.el_max))})
    zero_row = np.zeros(SECTORS, dtype=np.int64)
    if stats.windows:
        span = range(min(stats.windows), max(stats.windows) + 1)
        complete = [stats.windows.get(w, zero_row) for w in span][1:-1]
    else:
        complete = []
    if complete:
        cells = np.concatenate(complete)
        report["rays_per_sector_per_window"] = {
            "window_ms": window_ms, "windows": len(complete), "min": int(cells.min()),
            "p5": int(np.percentile(cells, 5)), "median": int(np.median(cells)),
            "max": int(cells.max()), "empty_cells": int((cells == 0).sum())}
    return report


def selftest():
    """Write a tiny synthetic LVX2 file with known content and check the report."""
    def package(ts, udp, points):
        payload = b"".join(struct.pack("<iiiBB", *p) for p in points)
        head = PACKAGE_HEADER.pack(0, 1, 9, 0, ts, udp, 1, len(payload), 0)
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
    if abs(report["range_m"]["max"] - 1.0) > 1e-9 or abs(report["elevation_deg"]["max"]) > 1e-9:
        fail("selftest range or elevation")
    with tempfile.NamedTemporaryFile(suffix=".lvx2") as handle:
        handle.write(blob + bytes(5))
        handle.flush()
        try:
            read_file(handle.name, 1.0)
        except SystemExit:
            pass
        else:
            fail("selftest: truncated file was accepted")
    cut = blob[:-(len(frames[-1]) - 10)]
    with tempfile.NamedTemporaryFile(suffix=".lvx2") as handle:
        handle.write(cut)
        handle.flush()
        try:
            read_file(handle.name, 1.0)
        except SystemExit:
            pass
        else:
            fail("selftest: file cut inside a frame header was accepted")
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
