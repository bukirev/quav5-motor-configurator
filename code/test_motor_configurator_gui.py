"""Тесты GUI-конфигуратора без подключения к плате (headless).

Проверяется логика приложения: чтение параметров, разбор наблюдений,
построение плана, таблицы и отчёта. Реальная плата не нужна.

Запуск: python test_motor_configurator_gui.py
"""
from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import motor_configurator as mc  # noqa: E402

FAILED = []


def check(name, condition, detail=""):
    if condition:
        print(f"  OK   {name}")
    else:
        print(f"  FAIL {name} {detail}")
        FAILED.append(name)


def fake_params(function_map, frame_class=4, frame_type=1):
    params = {}
    for output, function in function_map.items():
        params[f"SERVO{output}_FUNCTION"] = (function, 4)
        params[f"SERVO{output}_REVERSED"] = (0, 1)
        params[f"SERVO{output}_MIN"] = (1100, 4)
        params[f"SERVO{output}_MAX"] = (1900, 4)
    params["FRAME_CLASS"] = (frame_class, 4)
    params["FRAME_TYPE"] = (frame_type, 4)
    return params


def main():
    # Модальные окна в headless-прогоне заменяем заглушками: иначе
    # проверки блокировки (например, ARMED) висят на диалоге.
    _orig = (mc.messagebox.showerror, mc.messagebox.showinfo, mc.messagebox.showwarning)
    mc.messagebox.showerror = lambda *a, **k: None
    mc.messagebox.showinfo = lambda *a, **k: None
    mc.messagebox.showwarning = lambda *a, **k: None
    try:
        return _run()
    finally:
        mc.messagebox.showerror, mc.messagebox.showinfo, mc.messagebox.showwarning = _orig


def _run():
    app = mc.App()
    app.withdraw()

    # --- 1. параметры с платы разобраны верно ---
    function_map = {i: 32 + i for i in range(1, 9)}
    app._on_params(fake_params(function_map))
    check("прочитаны все выходы", len(app.current_function) == 8, app.current_function)
    check("M1 назначен на выход 1", app.current_function.get(1) == 33, app.current_function)
    check("класс рамы взят с платы", app.frame_class == 4, app.frame_class)
    check("тип рамы взят с платы", app.frame_type == 1, app.frame_type)
    check("таблица ожидания заполнена",
          len(app.expected_tree.get_children()) == 8,
          len(app.expected_tree.get_children()))

    # --- 2. наблюдения: правильная разводка -> план без изменений ---
    physical = {
        1: ("FR", "upper", "CCW"), 2: ("FL", "lower", "CW"),
        3: ("RL", "upper", "CCW"), 4: ("RR", "lower", "CW"),
        5: ("FL", "upper", "CCW"), 6: ("FR", "lower", "CW"),
        7: ("RR", "upper", "CCW"), 8: ("RL", "lower", "CW"),
    }
    for motor, (position, level, direction) in physical.items():
        app.observed_by_motor[motor] = (position, level, direction)
        app.slot_to_motor[(position, level)] = motor
    app.canvas.redraw()
    app._fill_obs_tree()
    app.rebuild_plan()
    check("наблюдения в таблице", len(app.obs_tree.get_children()) == 8,
          len(app.obs_tree.get_children()))
    check("план без ошибок", not app.plan.errors, [str(i) for i in app.plan.errors])
    check("менять нечего", app.plan.actions(app.current_function, app.current_reversed) == [],
          app.plan.actions(app.current_function, app.current_reversed))

    # --- 3. перепутанные выходы -> план предлагает обмен ---
    app.observed_by_motor.clear()
    app.slot_to_motor.clear()
    swapped = dict(physical)
    swapped[1] = ("FR", "lower", "CW")
    swapped[6] = ("FR", "upper", "CCW")
    for motor, (position, level, direction) in swapped.items():
        app.observed_by_motor[motor] = (position, level, direction)
        app.slot_to_motor[(position, level)] = motor
    app.rebuild_plan()
    actions = app.plan.actions(app.current_function, app.current_reversed)
    check("план предлагает два изменения", len(actions) == 2, actions)
    check("M1 уезжает на выход 6", app.plan.function_plan.get(6) == 33, app.plan.function_plan)
    check("M6 уезжает на выход 1", app.plan.function_plan.get(1) == 38, app.plan.function_plan)
    names = sorted(a[0] for a in actions)
    check("в изменениях только SERVOx_FUNCTION",
          names == ["SERVO1_FUNCTION", "SERVO6_FUNCTION"], names)
    check("в плане указано основание", all(len(a[3]) > 10 for a in actions), actions)

    # --- 4. одинаковое направление в паре -> ошибка ---
    app.observed_by_motor.clear()
    app.slot_to_motor.clear()
    broken = dict(physical)
    broken[6] = ("FR", "lower", "CCW")
    for motor, (position, level, direction) in broken.items():
        app.observed_by_motor[motor] = (position, level, direction)
        app.slot_to_motor[(position, level)] = motor
    app.rebuild_plan()
    check("ошибка про встречное вращение",
          any("одинаково" in i.text for i in app.plan.issues), [str(i) for i in app.plan.issues])
    check("план помечен как неприменимый", not app.plan.is_applicable)

    # --- 5. с коррекцией направления план становится применимым ---
    app.allow_reverse.set(True)
    app.rebuild_plan()
    check("предложен SERVO6_REVERSED", app.plan.reverse_plan.get(6) == 1, app.plan.reverse_plan)
    check("ошибок нет", not app.plan.errors, [str(i) for i in app.plan.errors])
    actions = app.plan.actions(app.current_function, app.current_reversed)
    check("в плане есть REVERSED для выхода 6",
          any(a[0] == "SERVO6_REVERSED" and a[2] == 1 for a in actions), actions)
    check("функция мотора не меняется (M6 остаётся на выходе 6)",
          app.plan.function_plan.get(6) == 38, app.plan.function_plan)

    # --- 6. таблица плана заполнена ---
    rows = app.plan_tree.get_children()
    check("в таблице плана 8 строк", len(rows) == 8, len(rows))
    values = [app.plan_tree.item(row, "values") for row in rows]
    check("в строке выхода 6 показан REVERSED",
          any(v[0] == "6" and v[4] == "0 -> 1" for v in values), values)
    check("в строке указано физическое место",
          any(v[0] == "6" and "передний правый" in v[5] for v in values), values)

    # --- 7. отчёт о записи формируется по шаблону ---
    app.report_rows = [("SERVO1_FUNCTION", 33, 38, "основание", "OK (прочитано 38)")]
    app._render_report(app.report_rows)
    text = app.report_text.get("1.0", "end")
    check("отчёт содержит параметр", "SERVO1_FUNCTION" in text, text[:120])

    # --- 8. экспорт .params не падает ---
    tmp_dir = os.path.join(os.environ.get("TEMP", "."), "motor_cfg_test")
    os.makedirs(tmp_dir, exist_ok=True)
    original_dir = mc.CONFIG_DIR
    mc.CONFIG_DIR = tmp_dir
    try:
        app.on_export_params()
        files = os.listdir(tmp_dir)
        check("файл параметров создан", any(f.endswith(".params") for f in files), files)
    finally:
        mc.CONFIG_DIR = original_dir

    # --- 9. несовпадение класса рамы на плате и в приложении ---
    app.frame_class = 4
    app.params["FRAME_CLASS"] = (3, 4)
    app.params["FRAME_TYPE"] = (1, 4)
    frame_actions = app._frame_actions()
    check("FRAME_CLASS попал в план изменений",
          any(a[0] == "FRAME_CLASS" and a[1] == 3 and a[2] == 4 for a in frame_actions),
          frame_actions)
    check("FRAME_TYPE не попал в план (совпадает)",
          not any(a[0] == "FRAME_TYPE" for a in frame_actions), frame_actions)
    app.params["FRAME_TYPE"] = (2, 4)
    frame_actions = app._frame_actions()
    check("FRAME_TYPE попал в план изменений",
          any(a[0] == "FRAME_TYPE" for a in frame_actions), frame_actions)

    # --- 10. проверки параметров выходов и моторов ---
    app.params["MOT_PWM_MIN"] = (1100, 4)
    app.params["MOT_PWM_MAX"] = (1900, 4)
    app.params["MOT_PWM_HOVER"] = (1500, 4)
    app.params["SERVO1_TRIM"] = (1500, 4)
    app.params["SERVO1_MIN"] = (1100, 4)
    app.params["SERVO1_MAX"] = (1900, 4)
    app.params["SERVO6_REVERSED"] = (1, 1)
    lines = app._check_output_params()
    check("диапазон газа признан корректным",
          any("MOT_PWM_MIN=1100" in line and line.startswith("OK") for line in lines), lines)
    check("инверсия выхода 6 отмечена",
          any("REVERSED=1" in line for line in lines), lines)
    app.params["MOT_PWM_MIN"] = (1900, 4)
    lines = app._check_output_params()
    check("некорректный диапазон газа найден",
          any(line.startswith("ERROR") for line in lines), lines)

    # --- 11. тест мотора отправляет номер ТЕСТА, а не номер мотора ---
    captured = {}

    class FakeLink:
        armed = False

        def motor_test(self, test_seq, percent=10.0, timeout=3.0, count=0, order=0):
            captured["test_seq"] = test_seq
            captured["percent"] = percent
            return {"result": 0, "command": 209, "progress": 0}

        def motor_test_stop(self):
            return None

    app.link = FakeLink()
    app.spin_ok.set(True)
    app.motor_box.current(5)          # M6
    app.on_motor_test()
    waited = 0.0
    while "test_seq" not in captured and waited < 3.0:
        time.sleep(0.05)
        waited += 0.05
    check("param1 = номер теста для M6 (2), а не 6",
          captured.get("test_seq") == 2, captured)
    app.motor_box.current(0)          # M1 (список: индекс 0 = M1)
    captured.clear()
    app.on_motor_test()
    waited = 0.0
    while "test_seq" not in captured and waited < 3.0:
        time.sleep(0.05)
        waited += 0.05
    check("param1 = 1 для M1", captured.get("test_seq") == 1, captured)
    app.motor_box.current(1)          # M2
    captured.clear()
    app.on_motor_test()
    waited = 0.0
    while "test_seq" not in captured and waited < 3.0:
        time.sleep(0.05)
        waited += 0.05
    check("param1 = 7 для M2", captured.get("test_seq") == 7, captured)
    # --- 12. кнопки A..H запускают тест мотора по номеру ТЕСТА ---
    captured.clear()

    class FakeLink2:
        armed = False

        def motor_test(self, test_seq, percent=10.0, timeout=3.0, count=0, order=0):
            captured["test_seq"] = test_seq
            return {"result": 0, "command": 209, "progress": 0}

        def motor_test_stop(self):
            return None

    app.link = FakeLink2()
    app.spin_ok.set(True)
    app.on_button_test("A")
    waited = 0.0
    while "test_seq" not in captured and waited < 3.0:
        time.sleep(0.05)
        waited += 0.05
    check("кнопка A -> тест №1 (M1)", captured.get("test_seq") == 1, captured)
    captured.clear()
    app.on_button_test("B")
    waited = 0.0
    while "test_seq" not in captured and waited < 3.0:
        time.sleep(0.05)
        waited += 0.05
    check("кнопка B -> тест №2 (M6)", captured.get("test_seq") == 2, captured)
    captured.clear()
    app.on_button_test("H")
    waited = 0.0
    while "test_seq" not in captured and waited < 3.0:
        time.sleep(0.05)
        waited += 0.05
    check("кнопка H -> тест №8 (M5)", captured.get("test_seq") == 8, captured)
    check("в интерфейсе 8 кнопок A..H", len(app.test_buttons) == 8, len(app.test_buttons))

    # --- 13. тест мотора блокируется при ARMED (§6) ---
    class ArmedLink(FakeLink2):
        armed = True

    app.link = ArmedLink()
    captured.clear()
    app.on_button_test("A")
    time.sleep(0.3)
    check("при ARMED тест не отправлен", "test_seq" not in captured, captured)
    app.link = FakeLink2()

    # --- 14. чек-лист проверки вращения: прогресс X/8 (§5.7) ---
    check("в чек-листе 8 строк", len(app.checklist_tree.get_children()) == 8,
          len(app.checklist_tree.get_children()))
    rows = app.checklist_tree.get_children()
    app.checklist_tree.selection_set(rows[0])
    app.checklist_result.set("совпало")
    app.on_save_checklist()
    check("в чек-листе 1 отмеченный мотор", app.checklist.get(1) == "совпало", app.checklist)
    progress = app.checklist_progress.cget("text")
    check("прогресс показывает 1/8", "1/8" in progress, progress)

    # --- 15. предупреждение о нестандартном типе рамы (§8) ---
    app.frame_class = 4
    app.frame_type = 14
    app._fill_expected()
    warning = app.frame_warning.get("1.0", "end")
    check("для CW_X показано предупреждение", "CW_X" in warning, warning[:200])
    app.frame_class = 3
    app._fill_expected()
    warning = app.frame_warning.get("1.0", "end")
    check("для OCTA показано предупреждение", "OCTA" in warning, warning[:200])
    app.frame_class = 4
    app.frame_type = 1
    app._fill_expected()
    warning = app.frame_warning.get("1.0", "end")
    check("для X8 предупреждений нет", "WARN" not in warning, warning[:200])

    app.link = None

    app.destroy()

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
