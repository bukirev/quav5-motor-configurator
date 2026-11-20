"""Download a PX4 parameter snapshot over MAVLink without modifying the vehicle.

Sends HEARTBEAT listening plus PARAM_REQUEST_LIST / PARAM_REQUEST_READ only.
No PARAM_SET, reboot, arm/disarm, or actuator command is sent.
Dependency: pyserial. Example:
  python download_px4_params.py --port COM18 --baud 115200
"""
from __future__ import annotations
import argparse
import datetime as dt
import struct
import time
from pathlib import Path

try:
    import serial
except ImportError:
    raise SystemExit("Missing pyserial. Install with: python -m pip install pyserial")

CRC_EXTRA = {0: 50, 20: 214, 21: 159, 22: 220}


def x25(data: bytes, crc: int = 0xFFFF) -> int:
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def v2_packet(seq: int, sysid: int, compid: int, msgid: int, payload: bytes) -> bytes:
    header = bytes((len(payload), 0, 0, seq & 0xFF, sysid & 0xFF, compid & 0xFF,
                    msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF))
    crc = x25(header + payload)
    crc = x25(bytes((CRC_EXTRA[msgid],)), crc)
    return b"\xFD" + header + payload + struct.pack("<H", crc)


def request_list(seq: int, target_sys: int, target_comp: int) -> bytes:
    return v2_packet(seq, 255, 190, 21, struct.pack("<BB", target_sys, target_comp))


def request_index(seq: int, target_sys: int, target_comp: int, index: int) -> bytes:
    payload = struct.pack("<hBB16s", index, target_sys, target_comp, b"\0" * 16)
    return v2_packet(seq, 255, 190, 20, payload)


def parse_frames(buffer: bytearray):
    """Yield validated (sysid, compid, msgid, payload) frames from a byte buffer."""
    while True:
        starts = [pos for pos in (buffer.find(b"\xFD"), buffer.find(b"\xFE")) if pos >= 0]
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
            received_crc = struct.unpack_from("<H", buffer, crc_offset)[0]
            valid = True
            if msgid in CRC_EXTRA:
                expected_crc = x25(bytes(buffer[1:10 + payload_len]))
                expected_crc = x25(bytes((CRC_EXTRA[msgid],)), expected_crc)
                valid = expected_crc == received_crc
            del buffer[:total]
            if valid:
                yield sysid, compid, msgid, payload
        elif magic == 0xFE:
            if len(buffer) < 6:
                return
            payload_len = buffer[1]
            total = 8 + payload_len
            if len(buffer) < total:
                return
            msgid = buffer[5]
            sysid, compid = buffer[3], buffer[4]
            payload = bytes(buffer[6:6 + payload_len])
            received_crc = struct.unpack_from("<H", buffer, 6 + payload_len)[0]
            valid = True
            if msgid in CRC_EXTRA:
                expected_crc = x25(bytes(buffer[1:6 + payload_len]))
                expected_crc = x25(bytes((CRC_EXTRA[msgid],)), expected_crc)
                valid = expected_crc == received_crc
            del buffer[:total]
            if valid:
                yield sysid, compid, msgid, payload


def decode_value(payload: bytes):
    if len(payload) < 25:
        return None
    raw_float, count, index, raw_name, param_type = struct.unpack_from("<fHH16sB", payload)
    name = raw_name.split(b"\0", 1)[0].decode("ascii", errors="replace")
    raw_value = payload[:4]
    if param_type == 9:
        value = raw_float
    elif param_type in (1, 3, 5, 7):
        value = int.from_bytes(raw_value, "little", signed=False)
    elif param_type in (2, 4, 6, 8):
        value = int.from_bytes(raw_value, "little", signed=True)
    else:
        value = raw_float
    return index, count, name, value, param_type


def qgc_value(value, param_type: int) -> str:
    if param_type == 9:
        return format(float(value), ".9g")
    return str(int(value))


def main():
    ap = argparse.ArgumentParser(description="Read a full PX4 parameter list and save a QGC/PX4 .params snapshot.")
    ap.add_argument("--port", default="COM18")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--outdir", default=r"C:\Trading\!!!px4\configs")
    ap.add_argument("--heartbeat-timeout", type=float, default=12.0)
    ap.add_argument("--download-timeout", type=float, default=150.0)
    args = ap.parse_args()

    try:
        link = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as exc:
        raise SystemExit(f"Cannot open {args.port}: {exc}")

    rx = bytearray()
    vehicle = None
    deadline = time.monotonic() + args.heartbeat_timeout
    while time.monotonic() < deadline and vehicle is None:
        rx.extend(link.read(4096))
        for sysid, compid, msgid, payload in parse_frames(rx):
            if msgid == 0 and len(payload) >= 9:
                _custom, vehicle_type, autopilot, _base_mode, _state, _version = struct.unpack_from("<IBBBBB", payload)
                vehicle = (sysid, compid, vehicle_type, autopilot)
                break

    if vehicle is None:
        link.close()
        raise SystemExit(f"No valid MAVLink heartbeat on {args.port} at {args.baud}; nothing was requested or saved.")

    sysid, autopilot_compid, vehicle_type, autopilot = vehicle
    print(f"Heartbeat: system={sysid}, component={autopilot_compid}, vehicle_type={vehicle_type}, autopilot={autopilot}")
    seq = 0
    link.write(request_list(seq, sysid, autopilot_compid))
    link.flush()

    expected_count = None
    received = {}
    last_progress = time.monotonic()
    hard_deadline = time.monotonic() + args.download_timeout
    while time.monotonic() < hard_deadline:
        rx.extend(link.read(8192))
        for r_sys, r_comp, msgid, payload in parse_frames(rx):
            if msgid != 22 or (r_sys, r_comp) != (sysid, autopilot_compid):
                continue
            item = decode_value(payload)
            if not item:
                continue
            index, count, name, value, param_type = item
            if expected_count is None:
                expected_count = count
            if count != expected_count:
                continue
            if 0 <= index < count:
                previous = received.get(index)
                current = (name, value, param_type)
                if previous != current:
                    received[index] = current
                    last_progress = time.monotonic()
        if expected_count is not None and len(received) >= expected_count:
            break
        if expected_count is not None and time.monotonic() - last_progress > 8:
            break

    missing = [] if expected_count is None else sorted(set(range(expected_count)) - set(received))
    # Retry missing indices individually (still read-only).
    for index in missing:
        seq = (seq + 1) & 0xFF
        link.write(request_index(seq, sysid, autopilot_compid, index))
        link.flush()
        time.sleep(0.015)
    if missing:
        end_retry = time.monotonic() + min(12.0, max(4.0, len(missing) * 0.15))
        while time.monotonic() < end_retry and len(received) < (expected_count or 0):
            rx.extend(link.read(8192))
            for r_sys, r_comp, msgid, payload in parse_frames(rx):
                if msgid != 22 or (r_sys, r_comp) != (sysid, autopilot_compid):
                    continue
                item = decode_value(payload)
                if item:
                    index, count, name, value, param_type = item
                    if count == expected_count and 0 <= index < count:
                        received[index] = (name, value, param_type)
    link.close()

    missing = [] if expected_count is None else sorted(set(range(expected_count)) - set(received))
    complete = expected_count is not None and not missing
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")
    suffix = "" if complete else ".partial"
    output = outdir / f"px4_params_{args.port}_{stamp}{suffix}.params"

    lines = [
        "# PX4 parameter snapshot (read-only MAVLink PARAM_REQUEST_LIST)",
        f"# Captured local time: {dt.datetime.now().astimezone().isoformat(timespec='seconds')}",
        f"# Port: {args.port} at {args.baud} baud; system {sysid}; component {autopilot_compid}",
        f"# Expected parameters: {expected_count}; received unique indices: {len(received)}; complete: {complete}",
        "# Vehicle and firmware identification should be verified separately.",
        "# Vehicle-Id Component-Id Name Value Type",
    ]
    for index in sorted(received):
        name, value, param_type = received[index]
        lines.append(f"{sysid} {autopilot_compid} {name} {qgc_value(value, param_type)} {param_type}")
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"Parameter count reported by PX4: {expected_count}")
    print(f"Unique parameter indices received: {len(received)}")
    print(f"Complete: {complete}")
    if missing:
        print(f"Missing indices ({len(missing)}): {missing[:40]}{' ...' if len(missing) > 40 else ''}")
    print(f"Saved: {output}")
    print("No parameter changes, reboot, arm/disarm, or actuator commands were sent.")
    if not complete:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
