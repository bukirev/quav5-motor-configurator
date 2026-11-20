"""Read-only PX4 MAVLink connection and selected parameter inspection.

This script only sends HEARTBEAT listening and PARAM_REQUEST_READ requests.
It does not change parameters or send actuator/motor commands.
Dependency: pyserial (python -m pip install pyserial)
Example: python mavlink_read_params.py --port COM18 --baud 115200
"""
from __future__ import annotations
import argparse
import struct
import sys
import time
from collections import Counter

try:
    import serial
except ImportError:
    raise SystemExit("Missing pyserial. Install with: python -m pip install pyserial")

# MAVLink CRC_EXTRA values for messages decoded/requested below.
CRC_EXTRA = {0: 50, 1: 124, 20: 214, 22: 220, 24: 24, 25: 87, 105: 93, 124: 87}
DEFAULT_PARAMS = [
    "SYS_AUTOSTART", "CA_ROTOR_COUNT",
    *[f"PWM_MAIN_FUNC{i}" for i in range(1, 9)],
    *[f"PWM_MAIN_MIN{i}" for i in range(1, 9)],
    *[f"PWM_MAIN_MAX{i}" for i in range(1, 9)],
    *[f"PWM_MAIN_DIS{i}" for i in range(1, 9)],
    "PWM_MAIN_TIM0", "PWM_MAIN_TIM1", "PWM_MAIN_TIM2",
    "UAVCAN_ENABLE", "UAVCAN_SUB_GPS", "UAVCAN_SUB_MAG",
    "GPS_1_CONFIG", "GPS_1_PROTOCOL",
    *[f"CAL_MAG{i}_ID" for i in range(3)],
    *[f"CAL_MAG{i}_PRIO" for i in range(3)],
]


def x25(data: bytes, crc: int = 0xFFFF) -> int:
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def make_param_request(seq: int, system_id: int, component_id: int, name: str) -> bytes:
    encoded = name.encode("ascii")
    if len(encoded) > 16:
        raise ValueError(f"MAVLink parameter names are limited to 16 bytes: {name}")
    payload = struct.pack("<hBB16s", -1, system_id, component_id, encoded.ljust(16, b"\0"))
    # MAVLink 2 header bytes after STX: len, incompat, compat, seq, sysid, compid, msgid[3]
    header = bytes((len(payload), 0, 0, seq & 0xFF, 255, 190, 20, 0, 0))
    checksum = x25(header + payload)
    checksum = x25(bytes((CRC_EXTRA[20],)), checksum)
    return b"\xFD" + header + payload + struct.pack("<H", checksum)


def extract_frames(buffer: bytearray):
    """Yield (version, sysid, compid, msgid, payload), skipping bad CRC frames."""
    while True:
        starts = [p for p in (buffer.find(b"\xFD"), buffer.find(b"\xFE")) if p >= 0]
        if not starts:
            buffer.clear()
            return
        start = min(starts)
        if start:
            del buffer[:start]
        if not buffer:
            return
        magic = buffer[0]
        if magic == 0xFD:
            if len(buffer) < 10:
                return
            payload_len = buffer[1]
            signed = bool(buffer[2] & 1)
            total = 12 + payload_len + (13 if signed else 0)
            if len(buffer) < total:
                return
            msgid = buffer[7] | (buffer[8] << 8) | (buffer[9] << 16)
            sysid, compid = buffer[5], buffer[6]
            payload = bytes(buffer[10:10 + payload_len])
            crc_offset = 10 + payload_len
            got_crc = struct.unpack_from("<H", buffer, crc_offset)[0]
            extra = CRC_EXTRA.get(msgid)
            valid = True
            if extra is not None:
                expected = x25(bytes(buffer[1:10 + payload_len]))
                expected = x25(bytes((extra,)), expected)
                valid = expected == got_crc
            del buffer[:total]
            if valid:
                yield (2, sysid, compid, msgid, payload)
        else:
            if len(buffer) < 6:
                return
            payload_len = buffer[1]
            total = 8 + payload_len
            if len(buffer) < total:
                return
            msgid = buffer[5]
            sysid, compid = buffer[3], buffer[4]
            payload = bytes(buffer[6:6 + payload_len])
            got_crc = struct.unpack_from("<H", buffer, 6 + payload_len)[0]
            extra = CRC_EXTRA.get(msgid)
            valid = True
            if extra is not None:
                expected = x25(bytes(buffer[1:6 + payload_len]))
                expected = x25(bytes((extra,)), expected)
                valid = expected == got_crc
            del buffer[:total]
            if valid:
                yield (1, sysid, compid, msgid, payload)


def decode_param_value(payload: bytes):
    if len(payload) < 25:
        return None
    raw_value, _count, _index, raw_name, param_type = struct.unpack_from("<fHH16sB", payload)
    name = raw_name.split(b"\0", 1)[0].decode("ascii", errors="replace")
    raw_union = payload[:4]
    if param_type == 9:  # MAV_PARAM_TYPE_REAL32
        value = raw_value
    elif param_type in (1, 3, 5, 7):  # unsigned integer types
        value = int.from_bytes(raw_union, "little", signed=False)
    elif param_type in (2, 4, 6, 8):  # signed integer types
        value = int.from_bytes(raw_union, "little", signed=True)
    else:
        value = raw_value
    return name, value, param_type


def main():
    parser = argparse.ArgumentParser(description="Read heartbeat, sensor status, and PX4 parameters over MAVLink.")
    parser.add_argument("--port", default="COM18", help="Serial port (default: COM18)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    parser.add_argument("--listen-seconds", type=float, default=4.0, help="Initial heartbeat wait")
    parser.add_argument("--response-seconds", type=float, default=5.0, help="Readback wait after parameter requests")
    args = parser.parse_args()

    try:
        link = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as exc:
        raise SystemExit(f"Cannot open {args.port}: {exc}")

    buffer = bytearray()
    counts = Counter()
    heartbeat = None
    sys_status = None
    gps = {}
    params = {}
    start = time.monotonic()
    while time.monotonic() - start < args.listen_seconds and heartbeat is None:
        buffer.extend(link.read(4096))
        for _version, sysid, compid, msgid, payload in extract_frames(buffer):
            counts[msgid] += 1
            if msgid == 0 and len(payload) >= 9:
                custom_mode, vehicle_type, autopilot, base_mode, system_state, _mavver = struct.unpack_from("<IBBBBB", payload)
                heartbeat = (sysid, compid, vehicle_type, autopilot, bool(base_mode & 128), system_state)
            elif msgid == 1 and len(payload) >= 12:
                sys_status = struct.unpack_from("<III", payload)
            elif msgid in (24, 124) and len(payload) >= 30:
                fix_type, sats = payload[28], payload[29]
                gps[msgid] = (fix_type, sats)

    if heartbeat is None:
        link.close()
        raise SystemExit(f"No valid MAVLink heartbeat on {args.port} at {args.baud}. No commands were sent.")

    system_id, autopilot_component, vehicle_type, autopilot, armed, system_state = heartbeat
    print(f"HEARTBEAT: sys={system_id} comp={autopilot_component} vehicle_type={vehicle_type} autopilot={autopilot} armed={armed} state={system_state}")
    print(f"RX_BYTES_BEFORE_QUERIES: messages={sum(counts.values())} ids={dict(counts)}")
    if sys_status:
        present, enabled, healthy = sys_status
        print(f"SYS_STATUS: present=0x{present:08X} enabled=0x{enabled:08X} healthy=0x{healthy:08X}")
    if gps:
        print("GPS_RAW: " + ", ".join(f"msg{mid}: fix_type={v[0]} satellites={v[1]}" for mid, v in sorted(gps.items())))

    # PARAM_REQUEST_READ is a read-only MAVLink request. There is no PARAM_SET or actuator command here.
    for seq, name in enumerate(DEFAULT_PARAMS):
        link.write(make_param_request(seq, system_id, autopilot_component, name))
        time.sleep(0.08)

    end = time.monotonic() + args.response_seconds
    while time.monotonic() < end:
        buffer.extend(link.read(8192))
        for _version, _sysid, _compid, msgid, payload in extract_frames(buffer):
            counts[msgid] += 1
            if msgid == 22:
                item = decode_param_value(payload)
                if item:
                    params[item[0]] = (item[1], item[2])
            elif msgid == 1 and len(payload) >= 12:
                sys_status = struct.unpack_from("<III", payload)
            elif msgid in (24, 124) and len(payload) >= 30:
                gps[msgid] = (payload[28], payload[29])

    link.close()
    print("PARAMETERS:")
    for name in DEFAULT_PARAMS:
        if name in params:
            value, param_type = params[name]
            print(f"  {name} = {value} (MAV_PARAM_TYPE={param_type})")
        else:
            print(f"  {name} = <no response>")
    if sys_status:
        present, enabled, healthy = sys_status
        print(f"SYS_STATUS_FINAL: present=0x{present:08X} enabled=0x{enabled:08X} healthy=0x{healthy:08X}")
    if gps:
        print("GPS_RAW_FINAL: " + ", ".join(f"msg{mid}: fix_type={v[0]} satellites={v[1]}" for mid, v in sorted(gps.items())))
    print("Read-only inspection complete; no parameters or actuator outputs were changed.")


if __name__ == "__main__":
    main()
