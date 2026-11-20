"""Кросс-проверка самодельного MAVLink-кодека против эталонной реализации.

Эталон генерируется официальным генератором pymavlink из XML-описания
MAVLink (common.xml), поэтому проверка независима от нашего кода.

Как получить эталон (нужен только для этой проверки):
    pip download pymavlink --no-binary :all: --no-deps
    # распаковать sdist, затем:
    python -m pymavlink.tools.mavgen --lang=Python3 --output=ref.py \
        message_definitions/v1.0/common.xml
    # и указать путь: set MAVLINK_REF=<путь к ref.py>

Если эталон не найден — проверки с ним пропускаются, остальные выполняются.

Запуск: python test_mavlink_codec.py
"""
from __future__ import annotations

import importlib.util
import os
import struct
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mavlink_ardupilot import (  # noqa: E402
    CRC_EXTRA,
    MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
    MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
    MAV_COMP_ID_ALL,
    MAV_REBOOT_FORCE_MAGIC,
    MSG_COMMAND_LONG,
    MSG_HEARTBEAT,
    MSG_PARAM_REQUEST_LIST,
    MSG_PARAM_SET,
    MSG_PARAM_VALUE,
    decode_autopilot_version,
    decode_param_value,
    extract_frames,
    pack_frame,
    x25,
)

FAILED = []


def check(name, condition, detail=""):
    if condition:
        print(f"  OK   {name}")
    else:
        print(f"  FAIL {name} {detail}")
        FAILED.append(name)


def load_reference():
    """Загрузить эталонный модуль pymavlink, если он доступен."""
    candidates = [
        os.environ.get("MAVLINK_REF"),
        os.path.join(tempfile.gettempdir(), "pymavlink_ref.py"),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "research", "mavlink", "pymavlink_ref.py"),
    ]
    for path in candidates:
        if path and os.path.isfile(path):
            spec = importlib.util.spec_from_file_location("mavlink_reference", path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            if hasattr(module, "MAVLink"):
                return module
    return None


class Collector:
    """Файл-приёмник для эталонной реализации."""

    def __init__(self):
        self.data = bytearray()

    def write(self, chunk):
        self.data.extend(chunk)

    def flush(self):
        pass

    def close(self):
        pass


def test_crc_table():
    print("CRC-16/MCRF4XX")
    check("x25('123456789') == 0x6F91", x25(b"123456789") == 0x6F91, hex(x25(b"123456789")))


def test_roundtrip_own():
    print("собственный кодек: сборка и разбор")
    payload = struct.pack("<fffffffHBBB", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 209, 1, 1, 0)
    frame = pack_frame(MSG_COMMAND_LONG, payload, 7, 255, 190)
    frames = list(extract_frames(bytearray(frame)))
    check("один кадр разобран", len(frames) == 1, len(frames))
    if frames:
        version, sysid, compid, msgid, body = frames[0]
        check("версия MAVLink2", version == 2, version)
        check("sysid/compid", (sysid, compid) == (255, 190), (sysid, compid))
        check("msgid", msgid == MSG_COMMAND_LONG, msgid)
        check("payload совпадает", body == payload)


def test_param_value_decode():
    print("разбор PARAM_VALUE")
    # ArduPilot кладёт число во float32 (GCS_Param.cpp: packet.param_value = param_value)
    raw = struct.pack("<fHH16sB", 33.0, 1153, 42, b"SERVO1_FUNCTION".ljust(16, b"\0"), 4)
    item = decode_param_value(raw)
    check("имя", item and item[0] == "SERVO1_FUNCTION", item)
    check("целое значение из float", item and item[1] == 33, item)
    check("тип", item and item[2] == 4, item)

    # вещественный параметр
    raw = struct.pack("<fHH16sB", 0.125, 10, 1, b"MOT_PWM_HOVER".ljust(16, b"\0"), 9)
    item = decode_param_value(raw)
    check("вещественное значение", item and abs(item[1] - 0.125) < 1e-9, item)

    # запасное соглашение: значение в байтах union'а (big-endian, вправо)
    raw = struct.pack(">xxh", 38) + struct.pack("<HH16sB", 1, 1, b"SERVO1_FUNCTION".ljust(16, b"\0"), 4)
    item = decode_param_value(raw)
    check("значение из байтов union'а", item and item[1] == 38, item)


def test_bad_crc_rejected():
    print("битый CRC отбрасывается")
    payload = struct.pack("<BB", 0, 0)
    frame = bytearray(pack_frame(MSG_PARAM_REQUEST_LIST, payload, 1, 255, 190))
    frame[-1] ^= 0xFF
    frames = list(extract_frames(bytearray(frame)))
    check("кадр отброшен", not frames, frames)


def test_against_reference():
    print("сверка с эталонной реализацией pymavlink")
    ref = load_reference()
    if ref is None:
        print("  SKIP эталон не найден (см. docstring: как получить)")
        return

    # 1) CRC: наша функция против эталонной
    sample = bytes(range(37))
    expected = ref._x25crc_slow()
    expected.accumulate(sample)
    check("CRC совпадает с эталоном", x25(sample) == expected.crc,
          f"{hex(x25(sample))} vs {hex(expected.crc)}")

    # 2) байты кадра: наш pack_frame против эталона
    #    PARAM_SET заканчивается ненулевым байтом (param_type), поэтому кадры
    #    должны совпасть побайтно.
    collector = Collector()
    mav = ref.MAVLink(collector, srcSystem=255, srcComponent=190)
    mav.send(mav.param_set_encode(1, 1, b"SERVO1_FUNCTION", 38.0, 9))
    theirs = bytes(collector.data)
    payload = struct.pack("<fBB16sB", 38.0, 1, 1, b"SERVO1_FUNCTION".ljust(16, b"\0"), 9)
    ours = pack_frame(MSG_PARAM_SET, payload, 0, 255, 190)
    check("PARAM_SET совпадает побайтно", ours == theirs,
          f"\n     ours  ={ours.hex()}\n     theirs={theirs.hex()}")

    # 3) наш парсер разбирает эталонный кадр
    frames = list(extract_frames(bytearray(theirs)))
    check("наш парсер разобрал эталонный кадр", len(frames) == 1, len(frames))
    if frames:
        _v, sysid, compid, msgid, body = frames[0]
        check("msgid совпал", msgid == MSG_PARAM_SET, msgid)
        check("sysid/compid совпали", (sysid, compid) == (255, 190), (sysid, compid))
        check("payload совпал", body == payload)

    # 4) COMMAND_LONG: эталон обрезает хвостовые нули (особенность MAVLink 2),
    #    поэтому сравниваем по существу, с дополнением нулями.
    collector = Collector()
    mav = ref.MAVLink(collector, srcSystem=255, srcComponent=190)
    mav.send(mav.command_long_encode(1, 1, 209, 0,
                                      1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0))
    theirs = bytes(collector.data)
    payload = struct.pack("<fffffffHBBB", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 209, 1, 1, 0)
    ours = pack_frame(MSG_COMMAND_LONG, payload, 0, 255, 190)
    our_frames = list(extract_frames(bytearray(ours)))
    their_frames = list(extract_frames(bytearray(theirs)))
    check("эталон собрал MAVLink2", theirs[:1] == b"\xFD", theirs[:1])
    check("наш кадр — MAVLink2", ours[:1] == b"\xFD", ours[:1])
    if our_frames and their_frames:
        check("COMMAND_LONG: msgid совпал",
              our_frames[0][3] == their_frames[0][3], (our_frames[0][3], their_frames[0][3]))
        check("COMMAND_LONG: sysid/compid совпали",
              our_frames[0][1:3] == their_frames[0][1:3])
        length = max(len(our_frames[0][4]), len(their_frames[0][4]))
        check("COMMAND_LONG: payload совпал",
              our_frames[0][4].ljust(length, b"\0") == their_frames[0][4].ljust(length, b"\0"),
              f"{our_frames[0][4].hex()} vs {their_frames[0][4].hex()}")

    # 5) HEARTBEAT
    collector = Collector()
    mav = ref.MAVLink(collector, srcSystem=255, srcComponent=190)
    mav.send(mav.heartbeat_encode(2, 3, 0, 0, 4))
    theirs = bytes(collector.data)
    payload = struct.pack("<IBBBBB", 0, 2, 3, 0, 4, 3)
    ours = pack_frame(MSG_HEARTBEAT, payload, 0, 255, 190)
    check("HEARTBEAT совпадает побайтно", ours == theirs,
          f"\n     ours  ={ours.hex()}\n     theirs={theirs.hex()}")

    # 6) CRC_EXTRA: наша таблица против класса сообщений эталона
    mismatches = []
    classes = [c for c in ref.MAVLink_message.__subclasses__() if c.__name__.startswith("MAVLink_")]
    for msgid, extra in CRC_EXTRA.items():
        for cls in classes:
            if getattr(cls, "id", None) == msgid and hasattr(cls, "crc_extra"):
                if cls.crc_extra != extra:
                    mismatches.append((msgid, extra, cls.crc_extra))
                break
    check("все CRC_EXTRA совпали с эталоном", not mismatches, mismatches)
    check("эталон содержит нужные сообщения", len(classes) > 100, len(classes))


def test_autopilot_version_decode():
    """Декодер MSG_AUTOPILOT_VERSION: формат кадра и разбор версии.

    Формат (pymavlink dialects/v20, mavlink_msg_autopilot_version_pack):
    <QQIIIIHH8B8B8B18B = 78 байт.
    Версия ArduPilot: major<<24 | minor<<16 | patch<<8 | fw_type
    (GCS_Common.cpp:3000-3003).
    """
    print("декодер AUTOPILOT_VERSION")
    # собираем payload как это делает ArduPilot
    fw = (4 << 24) | (7 << 16) | (2 << 8) | 3
    payload = struct.pack("<QQIIIIHH", 0x1234, 0xABCD, fw, 0, 0, 0, 0x1234, 0x5678)
    payload += b"Copter V4".ljust(8, b"\0")[:8]
    payload += b"\0" * 8 + b"\0" * 8
    payload += b"\0" * 18
    info = decode_autopilot_version(payload)
    check("кадр разобран", info is not None)
    if info:
        check("версия 4.7.2", info["version_text"] == "4.7.2", info["version_text"])
        check("major=4", info["version_major"] == 4, info["version_major"])
        check("minor=7", info["version_minor"] == 7, info["version_minor"])
        check("patch=2", info["version_patch"] == 2, info["version_patch"])
        check("тип прошивки=3", info["fw_type"] == 3, info["fw_type"])
        check("хеш прошивки прочитан", info["firmware_hash"] == "Copter V",
              info["firmware_hash"])
        check("board_version прочитан", info["board_version"] == 0, info["board_version"])
    # обрезанный хвост (MAVLink 2 отбрасывает нули) должен разбираться
    short = struct.pack("<QQIIIIHH", 0x1234, 0xABCD, fw, 0, 0, 0, 0x1234, 0x5678)
    info_short = decode_autopilot_version(short)
    check("обрезанный payload разобран", info_short is not None)
    if info_short:
        check("версия из обрезанного payload", info_short["version_text"] == "4.7.2",
              info_short["version_text"])
    check("слишком короткий payload -> None", decode_autopilot_version(b"\x00" * 8) is None)


def test_command_constants():
    """Константы команд: CMD 520 (версия) и CMD 246 (перезагрузка)."""
    print("константы команд")
    check("MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520",
          MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES == 520,
          MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES)
    check("MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246",
          MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN == 246,
          MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN)
    check("MAV_COMP_ID_ALL = 0", MAV_COMP_ID_ALL == 0, MAV_COMP_ID_ALL)
    check("магическое значение форс-перезагрузки = 20190226",
          MAV_REBOOT_FORCE_MAGIC == 20190226, MAV_REBOOT_FORCE_MAGIC)


def main():
    for fn in [
        test_crc_table,
        test_roundtrip_own,
        test_param_value_decode,
        test_bad_crc_rejected,
        test_against_reference,
        test_autopilot_version_decode,
        test_command_constants,
    ]:
        fn()
    print()
    if FAILED:
        print(f"ПРОВАЛЕНО проверок: {len(FAILED)}")
        for name in FAILED:
            print(f"  - {name}")
        return 1
    print("Все проверки пройдены.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
