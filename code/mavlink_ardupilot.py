"""MAVLink-канал для ArduCopter: параметры, тест мотора, телеметрия.

Read-only по умолчанию: запись параметров и тест мотора вызываются явно.

Зависимость: pyserial (python -m pip install pyserial)
Пример:
    python mavlink_ardupilot.py --port COM9 --list
    python mavlink_ardupilot.py --port COM9 --dump-params --out params.txt

CRC_EXTRA для используемых сообщений взят из официального диалекта MAVLink
(pymavlink 2.4.50, message_definitions/v1.0/common.xml + standard.xml + minimal.xml),
сгенерированного каталога dialects/v20.
"""
from __future__ import annotations

import math
import struct
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover - зависит от окружения
    raise SystemExit("Missing pyserial. Install with: python -m pip install pyserial")

# --- идентификаторы сообщений (pymavlink dialects/v20) ---
MSG_HEARTBEAT = 0
MSG_SYS_STATUS = 1
MSG_SYSTEM_TIME = 2
MSG_PARAM_REQUEST_READ = 20
MSG_PARAM_REQUEST_LIST = 21
MSG_PARAM_VALUE = 22
MSG_PARAM_SET = 23
MSG_GPS_RAW_INT = 24
MSG_ATTITUDE = 30
MSG_SERVO_OUTPUT_RAW = 36
MSG_COMMAND_INT = 75
MSG_COMMAND_LONG = 76
MSG_COMMAND_ACK = 77
MSG_VFR_HUD = 74
MSG_AUTOPILOT_VERSION = 148
MSG_EXTENDED_SYS_STATE = 245
MSG_STATUSTEXT = 253

# CRC_EXTRA: pymavlink 2.4.50 dialects/v20
CRC_EXTRA = {
    MSG_HEARTBEAT: 50,
    MSG_SYS_STATUS: 124,
    MSG_SYSTEM_TIME: 137,
    MSG_PARAM_REQUEST_READ: 214,
    MSG_PARAM_REQUEST_LIST: 159,
    MSG_PARAM_VALUE: 220,
    MSG_PARAM_SET: 168,
    MSG_GPS_RAW_INT: 24,
    MSG_ATTITUDE: 39,
    MSG_SERVO_OUTPUT_RAW: 222,
    MSG_COMMAND_LONG: 152,
    MSG_COMMAND_INT: 158,
    MSG_COMMAND_ACK: 143,
    MSG_VFR_HUD: 20,
    MSG_AUTOPILOT_VERSION: 178,
    MSG_EXTENDED_SYS_STATE: 130,
    MSG_STATUSTEXT: 83,
}

MAV_CMD_DO_MOTOR_TEST = 209
MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520
MAV_COMP_ID_ALL = 0
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_FAILED = 4
MAV_RESULT_DENIED = 2
MAV_RESULT_UNSUPPORTED = 3

# ArduPilot: магическое значение для перезагрузки при armed
# (GCS_Common.cpp:3646, handle_preflight_reboot). В COMMAND_LONG param6
# попадает в packet.y.
MAV_REBOOT_FORCE_MAGIC = 20190226

MAV_PARAM_TYPE_REAL32 = 9
MAV_PARAM_TYPE_UINT8 = 1
MAV_PARAM_TYPE_INT8 = 2
MAV_PARAM_TYPE_UINT16 = 3
MAV_PARAM_TYPE_INT16 = 4
MAV_PARAM_TYPE_UINT32 = 5
MAV_PARAM_TYPE_INT32 = 6

MAV_AUTOPILOT_ARDUPILOTMEGA = 3
MAV_AUTOPILOT_PX4 = 12

# ArduPilot: типы значения для MAV_CMD_DO_MOTOR_TEST (ArduCopter/motor_test.cpp)
MOTOR_TEST_THROTTLE_PERCENT = 0
MOTOR_TEST_THROTTLE_PWM = 1
MOTOR_TEST_THROTTLE_PILOT = 2
MOTOR_TEST_COMPASS_CAL = 3

# ArduPilot: порядок обхода моторов в тесте (MAVLink common.xml)
MOTOR_TEST_ORDER_DEFAULT = 0
MOTOR_TEST_ORDER_SEQUENCE = 1
MOTOR_TEST_ORDER_BOARD = 2


def x25(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/MCRF4XX, как в mavlink/mavcrc.py."""
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def pack_frame(msgid: int, payload: bytes, seq: int, sysid: int, compid: int) -> bytes:
    """Собрать MAVLink 2 кадр."""
    extra = CRC_EXTRA.get(msgid)
    if extra is None:
        raise ValueError(f"Нет CRC_EXTRA для сообщения {msgid}")
    header = bytes((
        len(payload), 0, 0, seq & 0xFF, sysid & 0xFF, compid & 0xFF,
        msgid & 0xFF, (msgid >> 8) & 0xFF, (msgid >> 16) & 0xFF,
    ))
    checksum = x25(header + payload)
    checksum = x25(bytes((extra,)), checksum)
    return b"\xFD" + header + payload + struct.pack("<H", checksum)


def extract_frames(buffer: bytearray):
    """Выдать корректные кадры из буфера, отбрасывая битые по CRC."""
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
            if len(buffer) < 12:
                return
            payload_len = buffer[1]
            total = 12 + payload_len
            if len(buffer) < total:
                return
            msgid = buffer[7] | (buffer[8] << 8) | (buffer[9] << 16)
            sysid, compid = buffer[5], buffer[6]
            payload = bytes(buffer[10:10 + payload_len])
            got_crc = struct.unpack_from("<H", buffer, 10 + payload_len)[0]
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
            if len(buffer) < 8:
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
    """PARAM_VALUE -> (имя, значение, тип) либо None.

    ArduPilot кладёт числовое значение параметра в float32 и отдельно шлёт
    param_type (GCS_Param.cpp: send_parameter_value -> packet.param_value = param_value).
    Для целочисленных типов значение читаем из float; если float не похож на
    целое, используем запасное соглашение MAVLink (значение в байтах union'а,
    big-endian, выровнено вправо — как делает pymavlink).
    """
    if len(payload) < 25:
        return None
    raw_value, _count, _index, raw_name, param_type = struct.unpack_from("<fHH16sB", payload)
    name = raw_name.split(b"\0", 1)[0].decode("ascii", errors="replace")
    value = raw_value
    if param_type in (1, 2, 3, 4, 5, 6, 7, 8):
        integral = (math.isfinite(raw_value)
                    and float(raw_value).is_integer()
                    and -(2 ** 31) < raw_value < 2 ** 31
                    and (raw_value == 0.0 or abs(raw_value) >= 1e-30))
        if integral:
            value = int(raw_value)
        else:
            value = _int_from_union(payload[:4], param_type)
    return name, value, param_type


def _int_from_union(raw: bytes, param_type: int) -> int:
    """Целое из байтов union'а param_value (соглашение MAVLink/pymavlink)."""
    if param_type in (1, 2):
        return int.from_bytes(raw[3:4], "big", signed=(param_type == 2))
    if param_type in (3, 4):
        return int.from_bytes(raw[2:4], "big", signed=(param_type == 4))
    return int.from_bytes(raw[0:4], "big", signed=(param_type == 6))


def decode_heartbeat(payload: bytes):
    if len(payload) < 9:
        return None
    custom_mode, vehicle_type, autopilot, base_mode, system_state, mavver = struct.unpack_from("<IBBBBB", payload)
    return {
        "custom_mode": custom_mode,
        "vehicle_type": vehicle_type,
        "autopilot": autopilot,
        "armed": bool(base_mode & 128),
        "system_state": system_state,
        "mavlink_version": mavver,
    }


def decode_autopilot_version(payload: bytes):
    """AUTOPILOT_VERSION -> dict с версией прошивки.

    Формат кадра (pymavlink dialects/v20, mavlink_msg_autopilot_version_pack):
    <QQIIIIHH8B8B8B18B = 78 байт, порядок полей:
      capabilities, uid, flight_sw_version, middleware_sw_version, os_sw_version,
      board_version, vendor_id, product_id, flight_custom_version[8],
      middleware_custom_version[8], os_custom_version[8], uid2[18].

    MAVLink 2 обрезает хвост payload из нулей, поэтому недостающие байты
    дополняем нулями (uid2 обычно весь нулевой).

    Версия прошивки у ArduPilot упакована как
    major<<24 | minor<<16 | patch<<8 | fw_type
    (GCS_Common.cpp:3000-3003, send_autopilot_version).
    """
    if len(payload) < 28:
        return None
    raw = payload.ljust(78, b"\0")
    (capabilities, uid, flight_sw, middleware_sw, os_sw,
     board_version, vendor_id, product_id) = struct.unpack_from("<QQIIIIHH", raw)
    major = (flight_sw >> 24) & 0xFF
    minor = (flight_sw >> 16) & 0xFF
    patch = (flight_sw >> 8) & 0xFF
    fw_type = flight_sw & 0xFF
    custom = raw[36:44].split(b"\0", 1)[0].decode("utf-8", errors="replace")
    return {
        "capabilities": capabilities,
        "uid": uid,
        "flight_sw_version": flight_sw,
        "version_text": f"{major}.{minor}.{patch}",
        "version_major": major,
        "version_minor": minor,
        "version_patch": patch,
        "fw_type": fw_type,
        "firmware_hash": custom,
        "middleware_sw_version": middleware_sw,
        "os_sw_version": os_sw,
        "board_version": board_version,
        "vendor_id": vendor_id,
        "product_id": product_id,
    }


def decode_command_ack(payload: bytes):
    if len(payload) < 10:
        return None
    command, result, progress, result_param2, target_system, target_component = struct.unpack_from("<HBBiBB", payload)
    return {
        "command": command,
        "result": result,
        "progress": progress,
        "result_param2": result_param2,
        "target_system": target_system,
        "target_component": target_component,
    }


def decode_statustext(payload: bytes):
    # формат кадра: <B50sHB
    if len(payload) < 2:
        return None
    severity = payload[0]
    text = payload[1:51].split(b"\0", 1)[0].decode("utf-8", errors="replace")
    return {"severity": severity, "text": text}


def decode_servo_output_raw(payload: bytes):
    if len(payload) < 21:
        return None
    time_usec = struct.unpack_from("<I", payload)[0]
    servos = list(struct.unpack_from("<8H", payload, 4))
    port = payload[20]
    return {"time_usec": time_usec, "servos": servos, "port": port}


class MAVLinkLink:
    """Один MAVLink-канал по последовательному порту."""

    def __init__(self, port: str, baud: int = 115200, source_system: int = 255, source_component: int = 190):
        self.port = port
        self.baud = baud
        self.source_system = source_system
        self.source_component = source_component
        self._serial = None
        self._buffer = bytearray()
        self._seq = 0
        self.system_id = None
        self.component_id = None
        self.autopilot = None
        self.vehicle_type = None
        self.armed = False
        self.firmware_version = None
        self.on_status = None  # callback(severity, text)

    # --- жизненный цикл -------------------------------------------------
    def open(self):
        self._serial = serial.Serial(self.port, self.baud, timeout=0.1)
        self._buffer = bytearray()

    def close(self):
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None

    @property
    def is_open(self):
        return self._serial is not None

    def _next_seq(self):
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def send(self, msgid: int, payload: bytes):
        if self._serial is None:
            raise RuntimeError("Соединение не открыто")
        frame = pack_frame(msgid, payload, self._next_seq(), self.source_system, self.source_component)
        self._serial.write(frame)

    # --- приём ----------------------------------------------------------
    def read_frames(self, timeout: float = 0.2):
        """Прочитать доступные кадры (до timeout)."""
        if self._serial is None:
            return []
        frames = []
        deadline = time.monotonic() + timeout
        while True:
            chunk = self._serial.read(4096)
            if chunk:
                self._buffer.extend(chunk)
                for frame in extract_frames(self._buffer):
                    frames.append(frame)
                    self._handle_common(frame)
                continue
            if time.monotonic() >= deadline:
                return frames

    def _handle_common(self, frame):
        _version, _sysid, _compid, msgid, payload = frame
        if msgid == MSG_HEARTBEAT:
            hb = decode_heartbeat(payload)
            if hb:
                self.autopilot = hb["autopilot"]
                self.vehicle_type = hb["vehicle_type"]
                self.armed = hb["armed"]
        elif msgid == MSG_STATUSTEXT:
            st = decode_statustext(payload)
            if st and self.on_status:
                self.on_status(st["severity"], st["text"])

    def wait_heartbeat(self, timeout: float = 10.0):
        """Ждать heartbeat. Возвращает dict с полями или None."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for _v, sysid, compid, msgid, payload in self.read_frames(0.25):
                if msgid == MSG_HEARTBEAT:
                    hb = decode_heartbeat(payload)
                    if hb:
                        self.system_id = sysid
                        self.component_id = compid
                        return hb
        return None

    # --- параметры ------------------------------------------------------
    def request_param_list(self, timeout: float = 30.0, progress=None):
        """Запросить все параметры. Возвращает dict имя -> (значение, тип)."""
        self.send(MSG_PARAM_REQUEST_LIST, struct.pack("<BB", 0, 0))
        params = {}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for _v, _s, _c, msgid, payload in self.read_frames(0.25):
                if msgid != MSG_PARAM_VALUE:
                    continue
                item = decode_param_value(payload)
                if item:
                    params[item[0]] = (item[1], item[2])
                    if progress:
                        progress(len(params))
        return params

    def read_param(self, name: str, timeout: float = 3.0):
        """Прочитать один параметр через PARAM_REQUEST_READ."""
        encoded = name.encode("ascii")[:16]
        payload = struct.pack("<hBB16s", -1, 0, 0, encoded.ljust(16, b"\0"))
        self.send(MSG_PARAM_REQUEST_READ, payload)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for _v, _s, _c, msgid, body in self.read_frames(0.2):
                if msgid != MSG_PARAM_VALUE:
                    continue
                item = decode_param_value(body)
                if item and item[0] == name:
                    return item[1], item[2]
        return None

    def set_param(self, name: str, value, timeout: float = 5.0):
        """Записать параметр (PARAM_SET) и дождаться эха PARAM_VALUE.

        ArduPilot игнорирует поле param_type и приводит float к типу самого
        параметра (GCS_MAVLink::handle_param_set -> AP_Param::set_float).
        Возвращает (значение с платы, тип) или None, если эха не было.
        """
        encoded = name.encode("ascii")[:16]
        payload = struct.pack("<fBB16sB", float(value), 0, 0, encoded.ljust(16, b"\0"), MAV_PARAM_TYPE_REAL32)
        self.send(MSG_PARAM_SET, payload)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for _v, _s, _c, msgid, body in self.read_frames(0.2):
                if msgid != MSG_PARAM_VALUE:
                    continue
                item = decode_param_value(body)
                if item and item[0] == name:
                    return item[1], item[2]
        return None

    # --- команды --------------------------------------------------------
    def send_command_long(self, command: int, params, target_system: int = 1, target_component: int = 1,
                          timeout: float = 3.0):
        """COMMAND_LONG; params = список из 7 float. Возвращает COMMAND_ACK или None.

        Формат кадра (pymavlink dialects/v20): <fffffffHBBB, 33 байта.
        """
        p = list(params) + [0.0] * 7
        payload = struct.pack("<fffffffHBBB", p[0], p[1], p[2], p[3], p[4], p[5], p[6],
                              command, target_system, target_component, 0)
        self.send(MSG_COMMAND_LONG, payload)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for _v, _s, _c, msgid, body in self.read_frames(0.2):
                if msgid == MSG_COMMAND_ACK:
                    ack = decode_command_ack(body)
                    if ack and ack["command"] == command:
                        return ack
        return None

    def motor_test(self, test_seq: int, throttle_percent: float = 10.0, timeout_s: float = 3.0,
                   count: int = 0, order: int = MOTOR_TEST_ORDER_DEFAULT):
        """Тест мотора по ArduCopter 4.7.2.

        MAV_CMD_DO_MOTOR_TEST (COMMAND_LONG): param1 = НОМЕР ТЕСТА (не номер мотора!),
        param2 = тип значения (0 = процент), param3 = значение, param4 = таймаут,
        param5 = число моторов в последовательности, param6 = порядок обхода.

        ArduPilot ищет мотор по номеру теста: AP_MotorsMatrix::_output_test_seq()
        сверяет `_test_order[i] == motor_seq` (AP_MotorsMatrix.cpp:466-470).
        Для OCTAQUAD/X порядок такой: M1->1, M6->2, M4->3, M7->4, M3->5, M8->6,
        M2->7, M5->8. Номер теста для нужного мотора даёт
        motor_mapping_core.test_order_for_motor().

        ArduPilot 4.7.2 конвертирует COMMAND_LONG в COMMAND_INT, поэтому param5
        попадает в x, а обработчик использует его как motor_count
        (ArduCopter/GCS_MAVLink_Copter.cpp:734: handle_MAV_CMD_DO_MOTOR_TEST).
        """
        return self.send_command_long(
            MAV_CMD_DO_MOTOR_TEST,
            [test_seq, MOTOR_TEST_THROTTLE_PERCENT, throttle_percent, timeout_s, count, order, 0.0],
        )

    def motor_test_stop(self):
        """Остановить тест: таймаут 0 -> ArduPilot останавливает моторы."""
        return self.send_command_long(
            MAV_CMD_DO_MOTOR_TEST,
            [0, MOTOR_TEST_THROTTLE_PERCENT, 0.0, 0.0, 0, 0, 0.0],
        )

    def request_autopilot_version(self, timeout: float = 5.0):
        """Запросить версию прошивки (MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES).

        ArduPilot 4.7.2: param1 должен быть равен 1, иначе MAV_RESULT_FAILED
        (GCS_Common.cpp:5082-5088). В ответ плата шлёт MSG_AUTOPILOT_VERSION.
        Возвращает распакованный словарь версии или None.
        """
        p = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        payload = struct.pack("<fffffffHBBB", p[0], p[1], p[2], p[3], p[4], p[5], p[6],
                              MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, self.system_id or 1,
                              self.component_id or 1, 0)
        self.send(MSG_COMMAND_LONG, payload)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for _v, _s, _c, msgid, body in self.read_frames(0.2):
                if msgid == MSG_AUTOPILOT_VERSION:
                    info = decode_autopilot_version(body)
                    if info:
                        self.firmware_version = info
                        return info
        return None

    def reboot(self, timeout: float = 5.0, force: bool = False):
        """Перезагрузить плату (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN).

        Требования ArduPilot 4.7.2 (GCS_Common.cpp:3510-3686):
          * param1 = 1 (перезагрузка) или 3 (в загрузчик), иначе UNSUPPORTED;
          * target_component должен быть MAV_COMP_ID_ALL (0) или compid платы,
            иначе MAV_RESULT_DENIED;
          * при soft_armed перезагрузка отклоняется (FAILED), если не подставлено
            магическое значение 20190226 в packet.y (для нас — param6);
          * ACK отправляется ДО перезагрузки.

        Обычно force не нужен: тест мотора снимается через motor_test_stop(),
        который возвращает soft_armed в false.
        """
        params = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if force:
            params[5] = float(MAV_REBOOT_FORCE_MAGIC)
        return self.send_command_long(
            MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            params,
            target_system=self.system_id or 1,
            target_component=MAV_COMP_ID_ALL,
            timeout=timeout,
        )

    def wait_disarmed(self, timeout: float = 10.0) -> bool:
        """Дождаться, пока аппарат снимет armed (нужно перед перезагрузкой).

        Состояние ARMED в heartbeat берётся из motors->armed()
        (GCS_MAVLink_Copter.cpp:52-53), а soft_armed, который блокирует
        перезагрузку, ставится самим тестом мотора (motor_test.cpp:169)
        и снимается в motor_test_stop() (motor_test.cpp:213).
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for _v, _s, _c, msgid, body in self.read_frames(0.25):
                if msgid == MSG_HEARTBEAT:
                    hb = decode_heartbeat(body)
                    if hb and not hb["armed"]:
                        return True
            if not self.armed:
                return True
        return not self.armed


def list_ports():
    """Список COM-портов: [(имя, описание), ...]."""
    try:
        from serial.tools import list_ports as _lp
    except ImportError:
        return []
    return [(p.device, p.description or "") for p in _lp.comports()]


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Проверка MAVLink-канала с ArduCopter.")
    parser.add_argument("--port", help="COM-порт, например COM9")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--list", action="store_true", help="Показать доступные порты и выйти")
    parser.add_argument("--dump-params", action="store_true", help="Скачать все параметры")
    parser.add_argument("--out", help="Файл для сохранения параметров")
    args = parser.parse_args()

    if args.list or not args.port:
        for name, desc in list_ports():
            print(f"{name}\t{desc}")
        if not args.port:
            print("Порт не задан. Укажите --port или используйте --list.", file=sys.stderr)
            return 1

    link = MAVLinkLink(args.port, args.baud)
    try:
        link.open()
    except Exception as exc:
        raise SystemExit(f"Не удалось открыть {args.port}: {exc}")
    hb = link.wait_heartbeat(10.0)
    if hb is None:
        link.close()
        raise SystemExit("Heartbeat не получен — это не MAVLink-устройство или порт/скорость неверны.")
    print(f"HEARTBEAT sys={link.system_id} comp={link.component_id} autopilot={hb['autopilot']} "
          f"armed={hb['armed']} type={hb['vehicle_type']}")
    if args.dump_params:
        params = link.request_param_list(30.0)
        print(f"Параметров получено: {len(params)}")
        lines = [f"{name}\t{value}" for name, (value, _t) in sorted(params.items())]
        if args.out:
            with open(args.out, "w", encoding="utf-8") as fh:
                fh.write("\n".join(lines) + "\n")
            print(f"Сохранено в {args.out}")
        else:
            print("\n".join(lines))
    link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
