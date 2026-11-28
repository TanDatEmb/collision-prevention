"""Read MID-360S FOV settings over the Livox SDK2 command protocol."""

import argparse
import binascii
import json
import secrets
import socket
import struct
import zlib


_HEADER = struct.Struct("<BBHIHBB6xHI")
_REPLY_PREFIX = struct.Struct("<BH")
_PARAMETER = struct.Struct("<HH")
_REGION = struct.Struct("<iiii4x")
_REGION_FIELDS = ("yaw_start_deg", "yaw_stop_deg", "pitch_start_deg", "pitch_stop_deg")
_REGION_KEYS = {0x0015: "fov_cfg0", 0x0016: "fov_cfg1"}
_ENABLE_KEY = 0x0017
_SERIAL_KEY = 0x8000
_INQUIRE = 0x0101


def decode_reply(packet, sequence, primary_sn=""):
    if len(packet) < _HEADER.size + _REPLY_PREFIX.size:
        raise ValueError("truncated FOV reply")
    *header, header_crc, payload_crc = _HEADER.unpack_from(packet)
    if header != [0xAA, 0, len(packet), sequence, _INQUIRE, 1, 1]:
        raise ValueError("unexpected FOV reply header")
    payload = packet[_HEADER.size:]
    if header_crc != binascii.crc_hqx(packet[:18], 0xFFFF) or payload_crc != zlib.crc32(payload):
        raise ValueError("invalid FOV reply checksum")
    status, count = _REPLY_PREFIX.unpack_from(payload)
    if status != 0:
        raise ValueError(f"lidar rejected FOV query (status {status})")

    settings = {}
    offset = _REPLY_PREFIX.size
    for _ in range(count):
        if offset + _PARAMETER.size > len(payload):
            raise ValueError("truncated FOV parameter header")
        key, size = _PARAMETER.unpack_from(payload, offset)
        offset += _PARAMETER.size
        value = payload[offset:offset + size]
        if len(value) != size:
            raise ValueError("truncated FOV parameter value")
        offset += size
        if key == _ENABLE_KEY:
            if size != 1 or value[0] > 3 or "fov_cfg_en" in settings:
                raise ValueError("invalid fov_cfg_en")
            settings["fov_cfg_en"] = value[0]
        elif key in _REGION_KEYS:
            name = _REGION_KEYS[key]
            if size != _REGION.size or name in settings:
                raise ValueError(f"invalid {name}")
            settings[name] = dict(zip(_REGION_FIELDS, _REGION.unpack(value)))
        elif key == _SERIAL_KEY:
            if size != 16 or "serial" in settings:
                raise ValueError("invalid lidar serial field")
            serial = value.split(b"\x00", 1)[0].decode("ascii")
            if not serial or not serial.isprintable():
                raise ValueError("invalid lidar serial value")
            settings["serial"] = serial
    if offset != len(payload):
        raise ValueError("unexpected trailing bytes in FOV reply")
    if "fov_cfg_en" not in settings:
        raise ValueError("lidar did not return fov_cfg_en")
    for bit, name in enumerate(_REGION_KEYS.values()):
        if settings["fov_cfg_en"] & (1 << bit) and name not in settings:
            raise ValueError(f"lidar did not return enabled region {name}")
    if primary_sn and settings.get("serial") != primary_sn:
        raise ValueError(f"reply did not identify requested primary serial {primary_sn}")
    return settings


def read_fov(config, host_ip, sensor_ip, primary_sn):
    device = config["Mid360s"]
    hosts = device["host_net_info"]
    if isinstance(hosts, dict):
        hosts = [{key: value for key, value in hosts.items() if key != "lidar_ip"}]
    candidates = [host for host in hosts if host.get("host_ip", host.get("cmd_data_ip")) == host_ip]
    matching = [
        host for host in candidates
        if isinstance(host.get("lidar_ip"), list) and sensor_ip in host["lidar_ip"]
    ]
    default = [host for host in candidates if not isinstance(host.get("lidar_ip"), list)]
    ports = {host["cmd_data_port"] for host in (matching or default)}
    if len(ports) != 1:
        raise ValueError("no unique MID-360S host command port in config")
    (host_port,) = ports
    sensor_port = device["lidar_net_info"]["cmd_data_port"]
    if any(type(port) is not int or not 1 <= port <= 65535 for port in (host_port, sensor_port)):
        raise ValueError("invalid MID-360S command port")

    sequence = secrets.randbelow(65536)
    payload = struct.pack("<HHHHHH", 4, 0, *_REGION_KEYS, _ENABLE_KEY, _SERIAL_KEY)
    header = (0xAA, 0, _HEADER.size + len(payload), sequence, _INQUIRE, 0, 0)
    header_crc = binascii.crc_hqx(_HEADER.pack(*header, 0, 0)[:18], 0xFFFF)
    packet = _HEADER.pack(*header, header_crc, zlib.crc32(payload)) + payload
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((host_ip, host_port))
        sock.connect((sensor_ip, sensor_port))
        sock.settimeout(3.0)
        sock.send(packet)
        return decode_reply(sock.recv(65535), sequence, primary_sn)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--host-ip", required=True)
    parser.add_argument("--sensor-ip", required=True)
    parser.add_argument("--primary-sn", default="")
    args = parser.parse_args()
    try:
        with open(args.config, encoding="utf-8") as stream:
            settings = read_fov(json.load(stream), args.host_ip, args.sensor_ip, args.primary_sn)
    except (OSError, ValueError, KeyError, OverflowError, TypeError, AttributeError) as error:
        print(json.dumps({"sensor_ip": args.sensor_ip, "fov_cfg_en": None, "error": str(error)}))
        return 1
    print(json.dumps({"sensor_ip": args.sensor_ip, **settings}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
