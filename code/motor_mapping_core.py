"""Логика маппинга моторов ArduCopter: модель рамы, план правки, проверки.

Модуль чистый: ни GUI, ни MAVLink. Всё, что здесь считается, можно проверить
тестами без подключения к плате.

Основание для правил:
  * SRV_Channel.h (ArduPilot 4.7.2): k_motor1 = 33 -> мотор N имеет функцию 32 + N;
  * AP_MotorsMatrix.cpp: раскладка рамы (угол, направление вращения);
  * AP_Motors_Class.h: значения FRAME_CLASS и FRAME_TYPE.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Dict, Iterable, List, Optional, Tuple

from ardupilot_frames import FRAME_CLASS, FRAME_LAYOUTS, FRAME_TYPE

# SRV_Channel.h: k_motor1 = 33, k_motor2 = 34, ... k_motor8 = 40
SERVO_FUNCTION_MOTOR1 = 33
SERVO_FUNCTION_MOTOR_MAX = 48  # до 16 моторов

POSITION_RU = {
    "FR": "передний правый",
    "FL": "передний левый",
    "RR": "задний правый",
    "RL": "задний левый",
    "F": "передний (по центру)",
    "B": "задний (по центру)",
    "R": "правый (по центру)",
    "L": "левый (по центру)",
}

LEVEL_RU = {"upper": "верхний", "lower": "нижний"}

DIRECTION_RU = {"CW": "по часовой стрелке", "CCW": "против часовой стрелки"}

LEVELS = ("upper", "lower")

# Профиль из спецификации §5.1: OCTO QUAD X (X8) — стандартная схема ArduPilot.
PROFILE_ID = "arducopter-4.7-octaquad-x"
PROFILE_NAME = "OCTO QUAD X (X8) — стандартная схема ArduPilot"
PROFILE_FRAME_CLASS = 4
PROFILE_FRAME_TYPE = 1

# Линейка прошивки, для которой проверена раскладка (AP_MotorsMatrix.cpp
# коммита fb773567896014da007bca58358704d4a0d8516f, тег Copter-4.7.2).
SUPPORTED_FIRMWARE_SERIES = "4.7"

# Буквы кнопок Motor Test: порядок теста 1 -> "A", 2 -> "B" ... 8 -> "H".
# Для OCTOQUAD/X это даёт A..H = M1, M6, M4, M7, M3, M8, M2, M5 — так же,
# как на официальной схеме ArduPilot (connect-escs-and-motors.html).
TEST_BUTTONS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"


@dataclass(frozen=True)
class FrameMotor:
    """Логический мотор: что ArduPilot ожидает и где он физически стоит.

    Поля разделены по §5.8 спецификации профиля `arducopter-4.7-octaquad-x`:
      motor_number      — номер M1..M8 в логике ArduPilot;
      physical_position — луч рамы (FR/FL/RR/RL);
      layer            — ярус ("upper"/"lower"), None для рам без ярусов;
      rotation         — направление вращения (CCW/CW);
      test_order       — param1 для MAV_CMD_DO_MOTOR_TEST (номер ТЕСТА, не мотора);
      test_button      — буква кнопки Motor Test (A..H);
      servo_function_id— функция SRV-канала (33..40 для M1..M8);
      physical_output  — MAIN-выход, на котором функция стоит на плате
                         (None, если не прочитано из конфигурации).
    """

    motor_number: int
    physical_position: str
    layer: Optional[str]
    rotation: str
    angle: float
    test_order: int
    test_button: str
    servo_function_id: int
    physical_output: Optional[int] = None

    @property
    def position_ru(self) -> str:
        return POSITION_RU.get(self.physical_position, self.physical_position)

    @property
    def direction_ru(self) -> str:
        return DIRECTION_RU.get(self.rotation, self.rotation)

    @property
    def layer_ru(self) -> str:
        return LEVEL_RU.get(self.layer, self.layer) if self.layer else "—"

    def describe(self) -> str:
        return (f"M{self.motor_number} — {self.position_ru}, {self.layer_ru}, "
                f"{self.direction_ru}")


@dataclass(frozen=True)
class Observation:
    """Наблюдение: что физически подключено к выходу MAIN."""

    output: int
    position: str
    level: str
    direction: str

    @property
    def position_ru(self) -> str:
        return POSITION_RU.get(self.position, self.position)

    @property
    def level_ru(self) -> str:
        return LEVEL_RU.get(self.level, self.level)

    @property
    def direction_ru(self) -> str:
        return DIRECTION_RU.get(self.direction, self.direction)

    def describe(self) -> str:
        return f"выход {self.output}: {self.position_ru}, {self.level_ru}, {self.direction_ru}"


@dataclass(frozen=True)
class Issue:
    severity: str  # ERROR | WARN | INFO
    text: str

    def __str__(self) -> str:
        return f"[{self.severity}] {self.text}"


@dataclass
class MappingPlan:
    frame_class: int
    frame_type: int
    expected: List[FrameMotor] = field(default_factory=list)
    observations: Dict[int, Observation] = field(default_factory=dict)  # выход -> наблюдение
    function_plan: Dict[int, int] = field(default_factory=dict)        # выход -> функция SERVOx
    reverse_plan: Dict[int, int] = field(default_factory=dict)         # выход -> SERVOx_REVERSED
    issues: List[Issue] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)

    @property
    def errors(self) -> List[Issue]:
        return [i for i in self.issues if i.severity == "ERROR"]

    @property
    def warnings(self) -> List[Issue]:
        return [i for i in self.issues if i.severity == "WARN"]

    @property
    def is_applicable(self) -> bool:
        """План применим, если нет ошибок и есть что менять."""
        return not self.errors and bool(self.function_plan)

    def actions(self, current_function: Dict[int, int], current_reversed: Dict[int, int]) -> List[Tuple[str, object, object, str]]:
        """Список (параметр, было, стало, зачем) для записи на плату."""
        result = []
        for output in sorted(self.function_plan):
            new_func = self.function_plan[output]
            old_func = current_function.get(output, 0)
            if old_func == new_func:
                continue
            motor = new_func - SERVO_FUNCTION_MOTOR1 + 1
            result.append((
                f"SERVO{output}_FUNCTION", old_func, new_func,
                f"на выход {output} посадить M{motor} ({POSITION_RU.get(self.expected[motor-1].physical_position, '')}"
                f" {DIRECTION_RU.get(self.expected[motor-1].rotation, '')})"
                if motor - 1 < len(self.expected) else f"на выход {output} посадить M{motor}",
            ))
        for output in sorted(self.reverse_plan):
            new_rev = self.reverse_plan[output]
            old_rev = current_reversed.get(output, 0)
            if old_rev == new_rev:
                continue
            result.append((
                f"SERVO{output}_REVERSED", old_rev, new_rev,
                f"инверсия сигнала на выходе {output} — мотор вращается не так, как ждёт рама",
            ))
        return result


def servo_function_for_motor(motor: int) -> int:
    """Функция SRV-канала для логического мотора (M1 -> 33)."""
    return SERVO_FUNCTION_MOTOR1 + motor - 1


def motor_for_servo_function(function: int) -> Optional[int]:
    """33 -> 1, 34 -> 2 ... None, если это не мотор."""
    if SERVO_FUNCTION_MOTOR1 <= function <= SERVO_FUNCTION_MOTOR_MAX:
        return function - SERVO_FUNCTION_MOTOR1 + 1
    return None


def test_order_for_motor(frame_class: int, frame_type: int, motor: int) -> Optional[int]:
    """Порядковый номер теста для мотора — то, что ждёт ArduPilot в param1.

    ArduPilot ищет мотор по номеру теста, а не по номеру мотора
    (AP_MotorsMatrix::_output_test_seq: `motor_enabled[i] && _test_order[i] == motor_seq`,
    AP_MotorsMatrix.cpp:466-470). Для OCTAQUAD/X: M1->1, M6->2, M4->3, M7->4,
    M3->5, M8->6, M2->7, M5->8.
    """
    try:
        motors = frame_motors(frame_class, frame_type)
    except KeyError:
        return None
    for item in motors:
        if item.motor_number == motor:
            return item.test_order
    return None


def angle_to_position(angle: float) -> str:
    """Угол из таблицы рамы -> название позиции.

    Считается ровно как в AP_MotorsMatrix::add_motor:
        roll  = cos(radians(angle + 90))
        pitch = cos(radians(angle))
    Отрицательный roll -> правая сторона, положительный pitch -> перед.
    """
    roll = math.cos(math.radians(angle + 90.0))
    pitch = math.cos(math.radians(angle))
    side = ""
    if roll < -0.2:
        side = "R"
    elif roll > 0.2:
        side = "L"
    is_front = pitch > 0.2
    is_rear = pitch < -0.2
    if side and is_front:
        return "F" + side
    if side and is_rear:
        return "R" + side
    if side:
        return side
    if is_front:
        return "F"
    if is_rear:
        return "B"
    return "B"


def derive_layers(motors: List[Tuple[float, str, int]]) -> Optional[List[str]]:
    """Ярус каждого мотора ("upper"/"lower") — None, если ярусов нет.

    В матрице ArduPilot коаксиальная пара на одном луче записана двумя
    моторами с ОДИНАКОВЫМ углом (AP_MotorsMatrix.cpp, add_motor). Верхним
    считается тот мотор пары, который в таблице раньше.

    Для OCTAQUAD/X это даёт верх M1..M4 и низ M5..M8 — так же на официальной
    схеме ArduPilot (m_04_01_octo_quad_x.svg, m_04_00_octo_quad_plus.svg,
    m_04_12_octo_quad_x_bf.svg: верх A,C,E,G / низ H,B,D,F по позициям).
    Для рам без коаксиальных пар (QUAD, OCTA и т.п.) ярус не определён.
    """
    groups: Dict[float, List[int]] = {}
    for index, (angle, _direction, _order) in enumerate(motors):
        groups.setdefault(angle, []).append(index)
    if not groups or any(len(items) != 2 for items in groups.values()):
        return None
    layers: List[Optional[str]] = [None] * len(motors)
    for items in groups.values():
        layers[items[0]] = "upper"
        layers[items[1]] = "lower"
    return layers  # type: ignore[return-value]


def frame_motors(frame_class: int, frame_type: int) -> List[FrameMotor]:
    """Ожидаемая раскладка моторов для класса и типа рамы."""
    layout = FRAME_LAYOUTS.get(frame_class)
    if layout is None:
        raise KeyError(f"Нет раскладки для FRAME_CLASS={frame_class}")
    motors = layout["types"].get(frame_type)
    if motors is None:
        raise KeyError(
            f"Класс рамы {FRAME_CLASS.get(frame_class, frame_class)} не поддерживает "
            f"FRAME_TYPE={frame_type} ({FRAME_TYPE.get(frame_type, '?')})"
        )
    layers = derive_layers(motors)
    result = []
    for index, (angle, direction, test_order) in enumerate(motors, start=1):
        result.append(FrameMotor(
            motor_number=index,
            physical_position=angle_to_position(angle),
            layer=layers[index - 1] if layers else None,
            rotation=direction,
            angle=angle,
            test_order=test_order,
            test_button=TEST_BUTTONS[test_order - 1] if 1 <= test_order <= len(TEST_BUTTONS) else "",
            servo_function_id=servo_function_for_motor(index),
            physical_output=None,
        ))
    return result


def attach_physical_outputs(motors: Iterable[FrameMotor],
                             current_function: Dict[int, int]) -> List[FrameMotor]:
    """Проставить каждому мотору выход MAIN, на котором стоит его функция.

    Источник данных — прочитанные с платы SERVOx_FUNCTION. Пока параметры не
    прочитаны, physical_output остаётся None (§5.8: поле nullable).
    """
    output_by_motor: Dict[int, int] = {}
    for output, function in sorted(current_function.items()):
        motor = motor_for_servo_function(function)
        if motor is not None:
            output_by_motor.setdefault(motor, output)
    return [
        replace(motor, physical_output=output_by_motor.get(motor.motor_number))
        for motor in motors
    ]


def output_map(current_function: Dict[int, int]) -> Dict[int, int]:
    """выход -> номер логического мотора (только те, где назначен мотор)."""
    mapping = {}
    for output, function in sorted(current_function.items()):
        motor = motor_for_servo_function(function)
        if motor is not None:
            mapping[output] = motor
    return mapping


def audit_motor_functions(current_function: Dict[int, int],
                           motor_count: int = 8) -> List[Issue]:
    """Диагностика назначений функций моторов (§5.5, приёмочный пункт §7).

    Показывает отсутствующие функции (33…33+N-1) и повторные назначения
    одной и той же функции на разные выходы MAIN.
    """
    issues: List[Issue] = []
    by_function: Dict[int, List[int]] = {}
    for output, function in sorted(current_function.items()):
        if motor_for_servo_function(function) is None:
            continue
        by_function.setdefault(int(function), []).append(output)

    for function in sorted(by_function):
        outputs = by_function[function]
        if len(outputs) > 1:
            motor = motor_for_servo_function(function)
            issues.append(Issue(
                "ERROR",
                f"Функция {function} (M{motor}) назначена сразу на выходы "
                f"{', '.join(str(o) for o in outputs)}. Один мотор нельзя посадить "
                "на два выхода — один из них лишний.",
            ))

    for motor in range(1, motor_count + 1):
        function = servo_function_for_motor(motor)
        if function not in by_function:
            issues.append(Issue(
                "WARN",
                f"Функция {function} (M{motor}) не назначена ни на один выход MAIN — "
                "мотор не будет крутиться.",
            ))
    return issues


def check_physical_pairs(observations: Iterable[Observation], severity: str = "ERROR") -> List[Issue]:
    """Проверка коаксиальных пар: на одном луче моторы должны вращаться встречно."""
    issues: List[Issue] = []
    by_position: Dict[str, List[Observation]] = {}
    for obs in observations:
        by_position.setdefault(obs.position, []).append(obs)
    for position, items in sorted(by_position.items()):
        if len(items) != 2:
            continue
        directions = {obs.direction for obs in items}
        if len(directions) == 1:
            issues.append(Issue(
                severity,
                f"На луче {POSITION_RU.get(position, position)} оба мотора крутятся одинаково "
                f"({DIRECTION_RU.get(items[0].direction, '')}): {items[0].describe()} и {items[1].describe()}. "
                "Пара должна вращаться встречно, иначе момент по yaw не гасится.",
            ))
    return issues


def build_plan(frame_class: int, frame_type: int,
               observed_by_motor: Dict[int, Tuple[str, str, str]],
               current_function: Dict[int, int],
               allow_reverse: bool = False) -> MappingPlan:
    """Собрать план маппинга.

    observed_by_motor: номер логического мотора -> (позиция, уровень, направление),
        то есть что пользователь увидел при тесте этого мотора.
    current_function: выход MAIN -> функция SRV-канала (как на плате сейчас).
    allow_reverse: разрешить предлагать SERVOx_REVERSED для моторов, которые крутятся
        не в ту сторону.
    """
    plan = MappingPlan(frame_class=frame_class, frame_type=frame_type)
    try:
        plan.expected = frame_motors(frame_class, frame_type)
    except KeyError as exc:
        plan.issues.append(Issue("ERROR", str(exc)))
        return plan

    func_to_output: Dict[int, int] = {}
    for output, function in sorted(current_function.items()):
        motor = motor_for_servo_function(function)
        if motor is None:
            continue
        if motor in func_to_output:
            plan.issues.append(Issue(
                "ERROR",
                f"Мотор M{motor} назначен сразу на выход {func_to_output[motor]} и выход {output}. "
                "Один логический мотор нельзя назначить на два выхода.",
            ))
            continue
        func_to_output[motor] = output

    # 1. наблюдения пользователя -> физические выходы
    for motor in sorted(observed_by_motor):
        position, level, direction = observed_by_motor[motor]
        output = func_to_output.get(motor)
        if output is None:
            plan.issues.append(Issue(
                "WARN",
                f"M{motor} не назначен ни на один выход (тест мотора ничего не крутил) — "
                "наблюдение не использовано.",
            ))
            continue
        plan.observations[output] = Observation(output, position, level, direction)

    plan.issues.extend(check_physical_pairs(
        plan.observations.values(),
        severity="WARN" if allow_reverse else "ERROR",
    ))

    # 2. распределить ожидаемые моторы по выходам
    used_outputs = set()
    unmatched_motors: List[FrameMotor] = []
    for motor in plan.expected:
        candidates = [obs for obs in plan.observations.values()
                      if obs.position == motor.physical_position
                      and obs.direction == motor.rotation
                      and obs.output not in used_outputs]
        if candidates:
            chosen = sorted(candidates, key=lambda o: o.output)[0]
            plan.function_plan[chosen.output] = servo_function_for_motor(motor.motor_number)
            used_outputs.add(chosen.output)
            if len(candidates) > 1:
                others = ", ".join("выход " + str(c.output) for c in candidates[1:])
                plan.issues.append(Issue(
                    "INFO",
                    f"Для M{motor.motor_number} ({motor.position_ru}, {motor.direction_ru}) подходит несколько "
                    f"выходов ({others}) — выбран выход {chosen.output}. При одинаковом направлении "
                    "вращения выбор не влияет на работу: моменты взаимно компенсируются.",
                ))
        else:
            unmatched_motors.append(motor)

    # 3. остаток: сопоставляем по лучу, при несовпадении направления предлагаем разворот
    leftover_outputs = [o for o in plan.observations.values() if o.output not in used_outputs]
    leftover_motors = list(unmatched_motors)
    outputs_by_position: Dict[str, List[Observation]] = {}
    for obs in leftover_outputs:
        outputs_by_position.setdefault(obs.position, []).append(obs)
    motors_by_position: Dict[str, List[FrameMotor]] = {}
    for motor in leftover_motors:
        motors_by_position.setdefault(motor.physical_position, []).append(motor)

    for position in sorted(set(outputs_by_position) | set(motors_by_position)):
        outs = sorted(outputs_by_position.get(position, []), key=lambda o: o.output)
        mots = sorted(motors_by_position.get(position, []), key=lambda m: m.motor_number)
        if not outs:
            continue
        if not mots:
            for obs in outs:
                plan.issues.append(Issue(
                    "ERROR",
                    f"Выход {obs.output} ({obs.position_ru}, {obs.direction_ru}) не соответствует ни одному "
                    f"мотору рамы {FRAME_CLASS.get(frame_class, frame_class)}/"
                    f"{FRAME_TYPE.get(frame_type, frame_type)}.",
                ))
            continue
        if len(outs) != len(mots):
            plan.issues.append(Issue(
                "ERROR",
                f"На луче {POSITION_RU.get(position, position)} не сходится число моторов и выходов "
                f"({len(outs)} выход(ов) против {len(mots)} мотор(ов) рамы) — проверьте наблюдения.",
            ))
            continue
        for obs, motor in zip(outs, mots):
            if allow_reverse:
                plan.function_plan[obs.output] = servo_function_for_motor(motor.motor_number)
                plan.reverse_plan[obs.output] = 1
                used_outputs.add(obs.output)
                plan.issues.append(Issue(
                    "WARN",
                    f"Выход {obs.output} ({obs.position_ru}) крутится {obs.direction_ru}, а M{motor.motor_number} "
                    f"на этом луче должен крутиться {motor.direction_ru}. Предложено "
                    f"SERVO{obs.output}_REVERSED=1 (инверсия сигнала ESC). Если физически возможно — "
                    "переверните пропеллер, это надёжнее инверсии.",
                ))
                plan.notes.append(
                    f"Выход {obs.output}: SERVO{obs.output}_REVERSED=1 -> M{motor.motor_number} "
                    f"({motor.position_ru}, {motor.direction_ru})."
                )
            else:
                plan.issues.append(Issue(
                    "ERROR",
                    f"Выход {obs.output} ({obs.position_ru}, {obs.direction_ru}) не подходит ни к одному "
                    f"мотору рамы: на этом луче ожидается {mots[0].direction_ru}. "
                    "Включите коррекцию направления через SERVOx_REVERSED или переверните пропеллер.",
                ))

    # 4. моторы, которым не нашлось выхода
    assigned_functions = set(plan.function_plan.values())
    for motor in plan.expected:
        if servo_function_for_motor(motor.motor_number) in assigned_functions:
            continue
        plan.issues.append(Issue(
            "ERROR",
            f"Не найден выход для M{motor.motor_number} ({motor.position_ru}, {motor.direction_ru}): "
            "на этом луче нет мотора с таким направлением вращения.",
        ))

    # 5. краткая сводка по ожиданиям
    for motor in plan.expected:
        plan.notes.append(f"Ожидается {motor.describe()} (порядок в тесте {motor.test_order}).")

    return plan


def current_state_rows(current_function: Dict[int, int], observations: Dict[int, Observation]) -> List[Tuple[int, int, str]]:
    """Текущее состояние платы: (выход, логический мотор, физическое место)."""
    rows = []
    for output in sorted(current_function):
        motor = motor_for_servo_function(current_function[output])
        obs = observations.get(output)
        where = obs.describe().split(":", 1)[1].strip() if obs else "не проверено"
        rows.append((output, motor if motor else 0, where))
    return rows
