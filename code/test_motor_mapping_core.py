"""Тесты логики маппинга моторов (без MAVLink и без GUI).

Запуск: python test_motor_mapping_core.py
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from motor_mapping_core import (  # noqa: E402
    Observation,
    angle_to_position,
    attach_physical_outputs,
    audit_motor_functions,
    build_plan,
    check_physical_pairs,
    frame_motors,
    motor_for_servo_function,
    servo_function_for_motor,
    test_order_for_motor,
)

FAILED = []


def check(name, condition, detail=""):
    if condition:
        print(f"  OK   {name}")
    else:
        print(f"  FAIL {name} {detail}")
        FAILED.append(name)


# --- эталонная физическая раскладка нашей окто-квадро -------------------------
# Выход MAIN -> (луч, уровень, направление).
# Луч FR: M1 (ожидается CCW) и M6 (ожидается CW); остальные лучи аналогично.
PHYSICAL = {
    1: ("FR", "upper", "CCW"),
    2: ("FL", "lower", "CW"),
    3: ("RL", "upper", "CCW"),
    4: ("RR", "lower", "CW"),
    5: ("FL", "upper", "CCW"),
    6: ("FR", "lower", "CW"),
    7: ("RR", "upper", "CCW"),
    8: ("RL", "lower", "CW"),
}


def identity_function(n=8):
    return {i: 32 + i for i in range(1, n + 1)}


def observe(physical, current_function):
    """Что пользователь увидит при тесте каждого логического мотора."""
    out_of_motor = {}
    for output, function in current_function.items():
        motor = motor_for_servo_function(function)
        if motor:
            out_of_motor[motor] = output
    observed = {}
    for motor, output in out_of_motor.items():
        if output in physical:
            observed[motor] = physical[output]
    return observed


def test_angle_to_position():
    print("angle_to_position")
    check("45 -> FR", angle_to_position(45) == "FR", angle_to_position(45))
    check("135 -> RR", angle_to_position(135) == "RR", angle_to_position(135))
    check("-135 -> RL", angle_to_position(-135) == "RL", angle_to_position(-135))
    check("-45 -> FL", angle_to_position(-45) == "FL", angle_to_position(-45))
    check("0 -> F", angle_to_position(0) == "F", angle_to_position(0))
    check("180 -> B", angle_to_position(180) == "B", angle_to_position(180))
    check("90 -> R", angle_to_position(90) == "R", angle_to_position(90))
    check("-90 -> L", angle_to_position(-90) == "L", angle_to_position(-90))


def test_servo_function_numbers():
    print("нумерация функций SRV-каналов")
    check("M1 -> 33", servo_function_for_motor(1) == 33)
    check("M8 -> 40", servo_function_for_motor(8) == 40)
    check("33 -> M1", motor_for_servo_function(33) == 1)
    check("40 -> M8", motor_for_servo_function(40) == 8)
    check("0 -> None", motor_for_servo_function(0) is None)
    check("201 -> None", motor_for_servo_function(201) is None)


def test_quad_x_layout():
    print("раскладка QUAD / X (AP_MotorsMatrix.cpp, ArduPilot 4.7.2)")
    motors = frame_motors(1, 1)
    check("4 мотора", len(motors) == 4, len(motors))
    # порядок взят прямо из исходника: 45/CCW, -135/CCW, -45/CW, 135/CW
    expect = {1: ("FR", "CCW"), 2: ("RL", "CCW"), 3: ("FL", "CW"), 4: ("RR", "CW")}
    for m in motors:
        check(f"M{m.motor_number} = {expect[m.motor_number][0]} {expect[m.motor_number][1]}",
              (m.physical_position, m.rotation) == expect[m.motor_number],
              f"получено {m.physical_position} {m.rotation}")


def test_octaquad_x_layout():
    print("раскладка OCTAQUAD / X (наша окто-квадро)")
    motors = frame_motors(4, 1)
    check("8 моторов", len(motors) == 8, len(motors))
    expect = {
        1: ("FR", "CCW"), 2: ("FL", "CW"), 3: ("RL", "CCW"), 4: ("RR", "CW"),
        5: ("FL", "CCW"), 6: ("FR", "CW"), 7: ("RR", "CCW"), 8: ("RL", "CW"),
    }
    for m in motors:
        check(f"M{m.motor_number} = {expect[m.motor_number][0]} {expect[m.motor_number][1]}",
              (m.physical_position, m.rotation) == expect[m.motor_number],
              f"получено {m.physical_position} {m.rotation}")
    for pos in ("FR", "FL", "RR", "RL"):
        dirs = sorted(m.rotation for m in motors if m.physical_position == pos)
        check(f"луч {pos}: пара встречная", dirs == ["CCW", "CW"], dirs)


def test_unsupported_frame_type():
    print("неподдерживаемая комбинация класса и типа")
    try:
        frame_motors(9, 1)  # COAX (вертолет) с типом X
        check("должно бросать KeyError", False)
    except KeyError:
        check("KeyError для COAX/X", True)


def test_plan_correct_wiring():
    print("план при правильной разводке")
    current = identity_function()
    observed = observe(PHYSICAL, current)
    plan = build_plan(4, 1, observed, current)
    check("нет ошибок", not plan.errors, [str(i) for i in plan.errors])
    check("M1 остался на выходе 1", plan.function_plan.get(1) == 33, plan.function_plan)
    check("M6 остался на выходе 6", plan.function_plan.get(6) == 38, plan.function_plan)
    check("менять ничего не нужно", plan.actions(current, {}) == [], plan.actions(current, {}))
    check("план применим", plan.is_applicable)


def test_plan_fixes_swapped_outputs():
    print("план чинит перепутанные выходы (M1 и M6 на одном луче)")
    current = identity_function()
    physical = dict(PHYSICAL)
    physical[1] = ("FR", "lower", "CW")   # выход 1 физически на моторе FR-CW
    physical[6] = ("FR", "upper", "CCW")  # выход 6 — на моторе FR-CCW
    observed = observe(physical, current)
    plan = build_plan(4, 1, observed, current)
    check("нет ошибок", not plan.errors, [str(i) for i in plan.errors])
    check("M1 уехал на выход 6", plan.function_plan.get(6) == 33, plan.function_plan)
    check("M6 уехал на выход 1", plan.function_plan.get(1) == 38, plan.function_plan)
    actions = plan.actions(current, {})
    names = sorted(a[0] for a in actions)
    check("два действия", len(actions) == 2, actions)
    check("меняются SERVO1_FUNCTION и SERVO6_FUNCTION",
          names == ["SERVO1_FUNCTION", "SERVO6_FUNCTION"], names)


def test_plan_detects_same_direction_pair():
    print("диагностика: оба мотора на луче крутятся одинаково")
    current = identity_function()
    physical = dict(PHYSICAL)
    physical[6] = ("FR", "lower", "CCW")  # второй мотор FR тоже CCW
    observed = observe(physical, current)
    plan = build_plan(4, 1, observed, current)
    check("есть ошибка про встречное вращение",
          any("одинаково" in i.text for i in plan.issues), [str(i) for i in plan.issues])
    check("план неприменим без коррекции", not plan.is_applicable)


def test_reverse_proposal():
    print("предложение SERVOx_REVERSED при неверном направлении")
    current = identity_function()
    physical = dict(PHYSICAL)
    physical[6] = ("FR", "lower", "CCW")
    observed = observe(physical, current)
    plan = build_plan(4, 1, observed, current, allow_reverse=True)
    check("предложен REVERSED для выхода 6", plan.reverse_plan.get(6) == 1, plan.reverse_plan)
    check("M6 назначен на выход 6", plan.function_plan.get(6) == 38, plan.function_plan)
    check("ошибок нет", not plan.errors, [str(i) for i in plan.errors])
    check("в плане есть предупреждение",
          any(i.severity == "WARN" for i in plan.issues), [str(i) for i in plan.issues])
    plan_no = build_plan(4, 1, observed, current, allow_reverse=False)
    check("без флага REVERSED не предлагается", not plan_no.reverse_plan, plan_no.reverse_plan)
    check("без флага REVERSED есть ошибка", plan_no.errors, [str(i) for i in plan_no.issues])


def test_plan_detects_missing_motor():
    print("диагностика: мотор не проверен")
    current = identity_function()
    observed = {1: ("FR", "upper", "CCW")}
    plan = build_plan(4, 1, observed, current)
    check("ошибка: мотор не найден", any("Не найден выход" in i.text for i in plan.issues),
          [str(i) for i in plan.issues])


def test_plan_detects_duplicate_function():
    print("диагностика: два выхода на один мотор")
    current = {1: 33, 2: 33, 3: 35, 4: 36, 5: 37, 6: 38, 7: 39, 8: 40}
    observed = observe(PHYSICAL, current)
    plan = build_plan(4, 1, observed, current)
    check("ошибка про двойное назначение",
          any("сразу на выход" in i.text for i in plan.issues), [str(i) for i in plan.issues])


def test_physical_pairs_helper():
    print("проверка коаксиальных пар")
    good = [Observation(1, "FR", "upper", "CCW"), Observation(6, "FR", "lower", "CW")]
    bad = [Observation(1, "FR", "upper", "CCW"), Observation(6, "FR", "lower", "CCW")]
    check("хорошая пара без замечаний", not check_physical_pairs(good))
    check("плохая пара даёт замечание", len(check_physical_pairs(bad)) == 1)


def test_actions_format():
    print("формат записи параметров")
    current = identity_function()
    physical = dict(PHYSICAL)
    physical[1] = ("FR", "lower", "CW")
    physical[6] = ("FR", "upper", "CCW")
    plan = build_plan(4, 1, observe(physical, current), current)
    actions = plan.actions(current, {i: 0 for i in range(1, 9)})
    for name, old, new, reason in actions:
        check(f"{name}: было {old} -> стало {new}", isinstance(reason, str) and len(reason) > 10,
              (name, old, new, reason))


def test_test_order_for_motor():
    print("номер теста для MAV_CMD_DO_MOTOR_TEST (param1)")
    # OCTAQUAD/X: M1->1, M6->2, M4->3, M7->4, M3->5, M8->6, M2->7, M5->8
    expected = {1: 1, 2: 7, 3: 5, 4: 3, 5: 8, 6: 2, 7: 4, 8: 6}
    for motor, order in expected.items():
        check(f"M{motor} -> тест №{order}",
              test_order_for_motor(4, 1, motor) == order,
              test_order_for_motor(4, 1, motor))
    check("для QUAD/X M1 -> тест №1", test_order_for_motor(1, 1, 1) == 1)
    check("несуществующий мотор -> None", test_order_for_motor(4, 1, 12) is None)
    check("неизвестная рама -> None", test_order_for_motor(99, 1, 1) is None)


def test_model_fields_per_spec():
    """§5.8: у мотора отдельные поля motorNumber/physicalPosition/layer/
    rotation/testOrder/testButton/servoFunctionId/physicalOutput."""
    print("модель мотора по §5.8")
    motors = frame_motors(4, 1)
    m1 = motors[0]
    check("motorNumber", m1.motor_number == 1, m1.motor_number)
    check("physicalPosition", m1.physical_position == "FR", m1.physical_position)
    check("layer", m1.layer == "upper", m1.layer)
    check("rotation", m1.rotation == "CCW", m1.rotation)
    check("testOrder", m1.test_order == 1, m1.test_order)
    check("testButton", m1.test_button == "A", m1.test_button)
    check("servoFunctionId", m1.servo_function_id == 33, m1.servo_function_id)
    check("physicalOutput до чтения конфигурации = None", m1.physical_output is None,
          m1.physical_output)
    check("у всех 8 моторов есть servoFunctionId 33..40",
          [m.servo_function_id for m in motors] == list(range(33, 41)),
          [m.servo_function_id for m in motors])


def test_layers_and_buttons():
    """Ярусы и буквы кнопок теста: сверка с официальными схемами ArduPilot."""
    print("ярусы и буквы кнопок")
    motors = frame_motors(4, 1)
    # m_04_01_octo_quad_x.svg: верх M1..M4 (A,G,E,C), низ M5..M8 (H,B,D,F)
    expect_layer = {1: "upper", 2: "upper", 3: "upper", 4: "upper",
                    5: "lower", 6: "lower", 7: "lower", 8: "lower"}
    expect_button = {1: "A", 2: "G", 3: "E", 4: "C", 5: "H", 6: "B", 7: "D", 8: "F"}
    for motor in motors:
        check(f"M{motor.motor_number} ярус {expect_layer[motor.motor_number]}",
              motor.layer == expect_layer[motor.motor_number], motor.layer)
        check(f"M{motor.motor_number} кнопка {expect_button[motor.motor_number]}",
              motor.test_button == expect_button[motor.motor_number], motor.test_button)
    # m_04_14_octo_quad_x_cw.svg: верх 1,3,5,7 (A,C,E,G), низ 2,4,6,8 (B,D,F,H)
    cw = frame_motors(4, 14)
    check("CW_X: верх M1,M3,M5,M7", [m.layer for m in cw] ==
          ["upper", "lower", "upper", "lower", "upper", "lower", "upper", "lower"],
          [m.layer for m in cw])
    check("CW_X: буквы A..H по порядку", [m.test_button for m in cw] == list("ABCDEFGH"),
          [m.test_button for m in cw])
    # у QUAD/X ярусов нет
    quad = frame_motors(1, 1)
    check("у QUAD/X ярус не определён", all(m.layer is None for m in quad),
          [m.layer for m in quad])


def test_attach_physical_outputs():
    """physicalOutput проставляется из прочитанных SERVOx_FUNCTION."""
    print("физический выход из конфигурации")
    motors = frame_motors(4, 1)
    current = {1: 33, 2: 34, 3: 35, 4: 36, 5: 37, 6: 38, 7: 39, 8: 40}
    attached = attach_physical_outputs(motors, current)
    check("M1 на выходе 1", attached[0].physical_output == 1, attached[0].physical_output)
    check("M8 на выходе 8", attached[7].physical_output == 8, attached[7].physical_output)
    swapped = {1: 38, 6: 33}
    attached2 = attach_physical_outputs(motors, swapped)
    check("при перестановке M1 стоит на выходе 6", attached2[0].physical_output == 6,
          attached2[0].physical_output)
    check("при перестановке M6 стоит на выходе 1", attached2[5].physical_output == 1,
          attached2[5].physical_output)
    check("M2 без назначения -> None", attached2[1].physical_output is None,
          attached2[1].physical_output)


def test_audit_motor_functions():
    """§5.5 / §7: показ отсутствующих и повторных назначений функций 33…40."""
    print("диагностика назначений функций 33..40")
    good = audit_motor_functions(identity_function(), 8)
    check("при правильном назначении замечаний нет", not good, [str(i) for i in good])
    missing = audit_motor_functions({1: 33, 2: 34}, 8)
    check("пропущены M3..M8", len(missing) == 6, [str(i) for i in missing])
    check("все пропуски — WARN", all(i.severity == "WARN" for i in missing),
          [(i.severity, i.text) for i in missing])
    dup = audit_motor_functions({1: 33, 2: 33, 3: 35, 4: 36, 5: 37, 6: 38, 7: 39, 8: 40}, 8)
    check("повтор функции 33 — ошибка", any(i.severity == "ERROR" for i in dup),
          [(i.severity, i.text) for i in dup])
    check("в тексте названы оба выхода", any("выходы 1, 2" in i.text for i in dup),
          [i.text for i in dup])


def main():
    for fn in [
        test_angle_to_position,
        test_servo_function_numbers,
        test_test_order_for_motor,
        test_quad_x_layout,
        test_octaquad_x_layout,
        test_unsupported_frame_type,
        test_plan_correct_wiring,
        test_plan_fixes_swapped_outputs,
        test_plan_detects_same_direction_pair,
        test_reverse_proposal,
        test_plan_detects_missing_motor,
        test_plan_detects_duplicate_function,
        test_physical_pairs_helper,
        test_actions_format,
        test_model_fields_per_spec,
        test_layers_and_buttons,
        test_attach_physical_outputs,
        test_audit_motor_functions,
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
