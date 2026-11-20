"""Конфигуратор маппинга моторов для ArduCopter (CUAV V5+ и другие платы).

Задача: правильно назначить логические моторы M1..M8 на физические выходы
MAIN так, чтобы миксер ArduPilot соответствовал реальной раме, и проверить
направление вращения коаксиальных пар.

Как это работает:
  1. Подключение по COM-порту, определение автопилота, чтение параметров.
  2. Пользователь выбирает класс и тип рамы (FRAME_CLASS / FRAME_TYPE).
  3. Для каждого мотора выполняется тест (MAV_CMD_DO_MOTOR_TEST); пользователь
     отмечает на виде сверху, какой физический мотор крутится и в какую сторону.
  4. Приложение строит план: какая функция должна стоять на каждом выходе,
     где нужен SERVOx_REVERSED, и где есть физическая ошибка раскладки.
  5. Применение: бэкап всех параметров -> запись -> readback каждого параметра
     -> отчёт "было -> стало -> основание -> проверка".

Зависимость: pyserial (python -m pip install pyserial). pymavlink не нужен.
Запуск:  python motor_configurator.py
"""
from __future__ import annotations

import math
import os
import queue
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import messagebox, ttk
from typing import Dict, List, Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ardupilot_frames import FRAME_CLASS, FRAME_TYPE  # noqa: E402
from mavlink_ardupilot import (  # noqa: E402
    MAV_AUTOPILOT_ARDUPILOTMEGA,
    MAV_AUTOPILOT_PX4,
    MAVLinkLink,
    list_ports,
)
from motor_mapping_core import (  # noqa: E402
    DIRECTION_RU,
    LEVEL_RU,
    POSITION_RU,
    PROFILE_FRAME_CLASS,
    PROFILE_FRAME_TYPE,
    PROFILE_ID,
    PROFILE_NAME,
    SUPPORTED_FIRMWARE_SERIES,
    Issue,
    attach_physical_outputs,
    audit_motor_functions,
    build_plan,
    frame_motors,
    motor_for_servo_function,
    test_order_for_motor,
)

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_DIR = os.path.join(PROJECT_ROOT, "configs")
LOG_DIR = os.path.join(PROJECT_ROOT, "logs")

# Позиции лучей для нашей окто-квадро: 4 луча, по 2 мотора (верхний/нижний).
ARM_ORDER = ["FR", "FL", "RL", "RR"]

# Угол луча в градусах, как в таблице ArduPilot (0 — нос, отсчёт по часовой стрелке).
ARM_ANGLE = {"FR": 45, "RR": 135, "RL": -135, "FL": -45}

ARM_COLOR = {
    "FR": "#d9534f", "FL": "#f0ad4e", "RL": "#5bc0de", "RR": "#5cb85c",
}

LEVELS = ("upper", "lower")


class FrameCanvas(tk.Canvas):
    """Вид сверху: 4 луча, на каждом верхний и нижний мотор."""

    def __init__(self, master, app):
        super().__init__(master, width=520, height=520, background="#f7f7f9", highlightthickness=1,
                         highlightbackground="#c8c8cc")
        self.app = app
        self.bind("<Button-1>", self._on_click)
        self.bind("<Configure>", lambda _e: self.redraw())

    # --- геометрия ---
    def slot_point(self, position, level):
        """Экранные координаты слота: верхний мотор — снаружи, нижний — ближе к центру."""
        cx, cy = self.winfo_width() / 2.0, self.winfo_height() / 2.0
        radius = self._radius()
        a = math.radians(ARM_ANGLE[position])
        ux, uy = math.sin(a), -math.cos(a)          # вдоль луча (нос вверх, право вправо)
        px, py = math.cos(a), math.sin(a)            # перпендикуляр к лучу
        if level == "upper":
            along, side = radius, -27.0
        else:
            along, side = radius * 0.74, 27.0
        return cx + ux * along + px * side, cy + uy * along + py * side

    def _radius(self):
        cx, cy = self.winfo_width() / 2.0, self.winfo_height() / 2.0
        return max(60.0, min(cx, cy) - 95)

    def _on_click(self, event):
        for position in ARM_ORDER:
            for level in LEVELS:
                x, y = self.slot_point(position, level)
                if (event.x - x) ** 2 + (event.y - y) ** 2 <= 30 ** 2:
                    self.app.on_slot_clicked(position, level)
                    return

    # --- отрисовка ---
    def redraw(self):
        self.delete("all")
        app = self.app
        cx, cy = self.winfo_width() / 2.0, self.winfo_height() / 2.0
        if cx < 50:
            return
        radius = self._radius()

        # лучи
        for position in ARM_ORDER:
            x, y = self.slot_point(position, "upper")
            self.create_line(cx, cy, x, y, fill=ARM_COLOR[position], width=14, capstyle="round")
            self.create_line(cx, cy, x, y, fill="white", width=3, dash=(3, 5))

        # метка носа
        self.create_polygon(cx, cy - 46, cx - 9, cy - 30, cx + 9, cy - 30, fill="#444444", outline="")
        self.create_text(cx, cy - 56, text="X+", font=("Segoe UI", 9, "bold"), fill="#444444")

        # ожидаемое направление по лучу
        expected_by_pos = {}
        try:
            for motor in frame_motors(app.frame_class, app.frame_type):
                expected_by_pos.setdefault(motor.physical_position, []).append(motor)
        except KeyError:
            pass

        for position in ARM_ORDER:
            for level in LEVELS:
                x, y = self.slot_point(position, level)
                motor = app.slot_to_motor.get((position, level))
                selected = app.selected_slot == (position, level)
                color = ARM_COLOR[position]
                fill = color if motor else "white"
                self.create_oval(x - 26, y - 26, x + 26, y + 26, fill=fill,
                                 outline=("#111111" if selected else color),
                                 width=3 if selected else 2)
                if motor:
                    output = app.output_of_motor(motor)
                    label = f"M{motor}" + (f"\nвыход {output}" if output else "")
                    self.create_text(x, y, text=label, font=("Segoe UI", 8, "bold"), fill="white")
                    direction = app.observed_by_motor.get(motor, ("", "", "CCW"))[2]
                    arrow = self._direction_arrow(x, y, direction)
                else:
                    self.create_text(x, y, text="?", font=("Segoe UI", 11), fill="#999999")
                    arrow = None
                if arrow:
                    self.create_arc(x - 34, y - 34, x + 34, y + 34, start=arrow[0], extent=arrow[1],
                                    style="arc", width=3, outline="#111111")
                    self.create_line(*arrow[2], fill="#111111", width=3)
                    self.create_polygon(arrow[3], fill="#111111", outline="")

            # подпись луча с ожидаемыми моторами — сдвинута наружу от центра
            motors = expected_by_pos.get(position, [])
            if motors:
                text = " / ".join(f"M{m.motor_number}:{m.rotation}({m.test_button})" for m in motors)
            else:
                text = "нет мотора рамы"
            base = ARM_ANGLE[position]
            ang = math.radians(base)
            lx = cx + math.sin(ang) * (radius + 56)
            ly = cy - math.cos(ang) * (radius + 56)
            self.create_text(lx, ly, text=text, font=("Segoe UI", 8), fill="#333333")

    @staticmethod
    def _direction_arrow(x, y, direction):
        """Дуга со стрелкой: направление вращения (вид сверху)."""
        if direction == "CCW":
            start, extent = 300, 120
        else:
            start, extent = 120, -120
        rad = math.radians(start + extent)
        ex, ey = x + 34 * math.cos(rad), y + 34 * math.sin(rad)
        tang = math.radians(start + extent + (90 if direction == "CCW" else -90))
        tip = (ex + 10 * math.cos(tang), ey + 10 * math.sin(tang))
        left = (ex + 7 * math.cos(tang + 2.4), ey + 7 * math.sin(tang + 2.4))
        right = (ex + 7 * math.cos(tang - 2.4), ey + 7 * math.sin(tang - 2.4))
        return start, extent, (ex, ey, tip), [tip, left, right]


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Конфигуратор маппинга моторов — ArduCopter")
        self.geometry("1340x860")
        self.minsize(1100, 700)

        self.link = None
        self.params = {}
        self.current_function = {}
        self.current_reversed = {}
        # наблюдение пользователя: логический мотор -> (позиция, уровень, направление)
        self.observed_by_motor = {}
        self.slot_to_motor = {}
        self.selected_slot = None
        self.frame_class = 4
        self.frame_type = 1
        self.plan = None
        self.queue = queue.Queue()
        self.busy = False
        self.report_rows = []
        self.link_lock = threading.Lock()
        # Снимок состояния ARMED ДО старта теста: сам тест мотора поднимает
        # armed на плате (motor_test.cpp:165-170), поэтому проверять armed
        # нужно по состоянию, зафиксированному до отправки команды.
        self.armed_before_test = False
        self.test_running = False
        # Результаты ручной проверки вращения: номер мотора -> "совпало"/...
        self.checklist: Dict[int, str] = {}
        self.firmware_version = None

        self._build_ui()
        self.after(100, self._poll)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- интерфейс ----------------
    def _build_ui(self):
        top = ttk.Frame(self, padding=(8, 6))
        top.pack(side="top", fill="x")
        ttk.Label(top, text="Порт:").pack(side="left")
        self.port_box = ttk.Combobox(top, width=10, values=[p for p, _ in list_ports()])
        self.port_box.pack(side="left", padx=(4, 10))
        ttk.Label(top, text="Baud:").pack(side="left")
        self.baud_box = ttk.Combobox(top, width=8, values=["115200", "57600", "230400", "460800", "921600"])
        self.baud_box.set("115200")
        self.baud_box.pack(side="left", padx=(4, 10))
        ttk.Button(top, text="Подключиться", command=self.on_connect).pack(side="left")
        ttk.Button(top, text="Отключиться", command=self.on_disconnect).pack(side="left", padx=4)
        ttk.Button(top, text="Прочитать все параметры", command=self.on_read_params).pack(side="left", padx=4)
        ttk.Button(top, text="Обновить порты", command=self.on_refresh_ports).pack(side="left", padx=4)
        self.status_label = ttk.Label(top, text="Нет соединения", foreground="#a00")
        self.status_label.pack(side="left", padx=14)
        self.info_label = ttk.Label(top, text="")
        self.info_label.pack(side="left")

        body = ttk.PanedWindow(self, orient="horizontal")
        body.pack(side="top", fill="both", expand=True, padx=8)

        left = ttk.Frame(body)
        body.add(left, weight=1)
        ttk.Label(left, text="Рама сверху: нос — вверх, X+ — вперёд",
                  font=("Segoe UI", 9, "bold")).pack(anchor="w", pady=(6, 2))
        self.canvas = FrameCanvas(left, self)
        self.canvas.pack(fill="both", expand=True)

        right = ttk.Notebook(body)
        body.add(right, weight=2)

        # --- вкладка "Рама" ---
        tab_frame = ttk.Frame(right, padding=8)
        right.add(tab_frame, text="Рама")
        row = ttk.Frame(tab_frame)
        row.pack(fill="x")
        ttk.Label(row, text="FRAME_CLASS:").pack(side="left")
        self.class_box = ttk.Combobox(row, width=34, state="readonly",
                                      values=[f"{v} — {FRAME_CLASS[v]}" for v in sorted(FRAME_CLASS)])
        self.class_box.current(4)
        self.class_box.pack(side="left", padx=4)
        self.class_box.bind("<<ComboboxSelected>>", self.on_frame_change)
        ttk.Label(row, text="FRAME_TYPE:").pack(side="left", padx=(14, 0))
        self.type_box = ttk.Combobox(row, width=18, state="readonly",
                                     values=[f"{v} — {FRAME_TYPE[v]}" for v in sorted(FRAME_TYPE)])
        self.type_box.current(1)
        self.type_box.pack(side="left", padx=4)
        self.type_box.bind("<<ComboboxSelected>>", self.on_frame_change)

        ttk.Label(tab_frame, text="Ожидаемая раскладка (из исходников ArduPilot 4.7.2):",
                  font=("Segoe UI", 9, "bold")).pack(anchor="w", pady=(10, 4))
        self.expected_tree = ttk.Treeview(tab_frame,
                                           columns=("motor", "pos", "level", "dir", "order",
                                                    "button", "func", "out"),
                                           show="headings", height=10)
        for col, title, width in (("motor", "Мотор", 60), ("pos", "Положение", 150),
                                  ("level", "Ярус", 70), ("dir", "Вращение", 140),
                                  ("order", "Тест №", 60), ("button", "Кнопка", 60),
                                  ("func", "Функция", 70), ("out", "Выход", 60)):
            self.expected_tree.heading(col, text=title)
            self.expected_tree.column(col, width=width, anchor="w")
        self.expected_tree.pack(fill="both", expand=False)
        self.expected_tree.tag_configure("odd", background="#f2f2f2")

        self.profile_label = ttk.Label(tab_frame, text="", font=("Segoe UI", 9, "bold"))
        self.profile_label.pack(anchor="w", pady=(8, 0))
        self.frame_warning = tk.Text(tab_frame, height=4, wrap="word", font=("Consolas", 9))
        self.frame_warning.pack(fill="x", pady=(4, 0))

        # --- вкладка "Идентификация" ---
        tab_ident = ttk.Frame(right, padding=8)
        right.add(tab_ident, text="Идентификация")
        ttk.Label(tab_ident, text="Шаг 1. Крутим мотор и смотрим, что крутится:",
                  font=("Segoe UI", 9, "bold")).pack(anchor="w")
        ident_row = ttk.Frame(tab_ident)
        ident_row.pack(fill="x", pady=4)
        ttk.Label(ident_row, text="Мотор:").pack(side="left")
        self.motor_box = ttk.Combobox(ident_row, width=10, state="readonly",
                                      values=[f"M{i}" for i in range(1, 17)])
        self.motor_box.current(0)
        self.motor_box.pack(side="left", padx=4)
        ttk.Label(ident_row, text="Газ:").pack(side="left", padx=(12, 0))
        self.throttle = ttk.Spinbox(ident_row, from_=1, to=50, width=6)
        self.throttle.set(10)
        self.throttle.pack(side="left", padx=4)
        ttk.Label(ident_row, text="с").pack(side="left")
        self.spin_ok = tk.BooleanVar(value=False)
        ttk.Checkbutton(ident_row, text="Пропеллеры сняты, аппарат на земле",
                        variable=self.spin_ok).pack(side="left", padx=(14, 0))
        ttk.Button(ident_row, text="Крутить", command=self.on_motor_test).pack(side="left", padx=8)

        ttk.Label(tab_ident, text="Шаг 2. Отмечаем на виде сверху, какой мотор крутился "
                                  "(клик по кружку) и его направление:",
                  font=("Segoe UI", 9, "bold")).pack(anchor="w", pady=(10, 2))
        dir_row = ttk.Frame(tab_ident)
        dir_row.pack(fill="x")
        ttk.Label(dir_row, text="Направление вращения:").pack(side="left")
        self.direction = ttk.Combobox(dir_row, width=26, state="readonly",
                                      values=["CCW — против часовой", "CW — по часовой"])
        self.direction.current(0)
        self.direction.pack(side="left", padx=4)
        self.level = ttk.Combobox(dir_row, width=16, state="readonly",
                                  values=["верхний", "нижний"])
        self.level.current(0)
        self.level.pack(side="left", padx=8)
        ttk.Button(dir_row, text="Сохранить наблюдение", command=self.on_save_observation).pack(side="left", padx=6)
        ttk.Button(dir_row, text="Очистить всё", command=self.on_clear_observations).pack(side="left")

        ttk.Label(tab_ident, text="Motor Test: кнопки A…H соответствуют порядку теста "
                                  "ArduPilot (param1 = номер теста, не номер мотора):",
                  font=("Segoe UI", 9, "bold")).pack(anchor="w", pady=(12, 2))
        btn_row = ttk.Frame(tab_ident)
        btn_row.pack(fill="x")
        self.test_buttons = {}
        for letter in "ABCDEFGH":
            btn = ttk.Button(btn_row, text=letter, width=4,
                             command=lambda ltr=letter: self.on_button_test(ltr))
            btn.pack(side="left", padx=2)
            self.test_buttons[letter] = btn
        self.test_button_hint = ttk.Label(btn_row, text="")
        self.test_button_hint.pack(side="left", padx=(10, 0))

        ttk.Label(tab_ident, text="Чек-лист проверки вращения (X/8 проверено):",
                  font=("Segoe UI", 9, "bold")).pack(anchor="w", pady=(12, 2))
        self.checklist_tree = ttk.Treeview(tab_ident,
                                            columns=("motor", "pos", "level", "dir", "result"),
                                            show="headings", height=8)
        for col, title, width in (("motor", "Мотор", 60), ("pos", "Положение", 150),
                                  ("level", "Ярус", 70), ("dir", "Ожид. вращение", 130),
                                  ("result", "Результат проверки", 220)):
            self.checklist_tree.heading(col, text=title)
            self.checklist_tree.column(col, width=width, anchor="w")
        self.checklist_tree.pack(fill="both", expand=False)
        self.checklist_tree.tag_configure("ok", background="#e8f5e9")
        self.checklist_tree.tag_configure("bad", background="#f8d7da")
        checklist_row = ttk.Frame(tab_ident)
        checklist_row.pack(fill="x", pady=(4, 0))
        ttk.Label(checklist_row, text="Отметить мотор как проверенный:").pack(side="left")
        self.checklist_result = ttk.Combobox(checklist_row, width=20, state="readonly",
                                             values=["совпало", "не совпало", "не проверено"])
        self.checklist_result.current(2)
        self.checklist_result.pack(side="left", padx=4)
        ttk.Button(checklist_row, text="Записать результат",
                   command=self.on_save_checklist).pack(side="left")
        self.checklist_progress = ttk.Label(checklist_row, text="")
        self.checklist_progress.pack(side="left", padx=10)

        ttk.Label(tab_ident, text="Наблюдения:", font=("Segoe UI", 9, "bold")).pack(anchor="w", pady=(10, 2))
        self.obs_tree = ttk.Treeview(tab_ident, columns=("motor", "out", "pos", "level", "dir"),
                                      show="headings", height=10)
        for col, title, width in (("motor", "Мотор", 70), ("out", "Выход", 70), ("pos", "Положение", 150),
                                  ("level", "Уровень", 90), ("dir", "Направление", 140)):
            self.obs_tree.heading(col, text=title)
            self.obs_tree.column(col, width=width, anchor="w")
        self.obs_tree.pack(fill="both", expand=True)

        # --- вкладка "План" ---
        tab_plan = ttk.Frame(right, padding=8)
        right.add(tab_plan, text="План и применение")
        self.plan_tree = ttk.Treeview(tab_plan, columns=("out", "cur", "plan", "motor", "rev", "phys"),
                                       show="headings", height=9)
        for col, title, width in (("out", "Выход MAIN", 100), ("cur", "Функция сейчас", 120),
                                  ("plan", "Функция в плане", 130), ("motor", "Мотор", 70),
                                  ("rev", "REVERSED", 130), ("phys", "Физически", 240)):
            self.plan_tree.heading(col, text=title)
            self.plan_tree.column(col, width=width, anchor="w")
        self.plan_tree.pack(fill="x")
        self.plan_tree.tag_configure("change", background="#fff3cd")
        self.plan_tree.tag_configure("same", background="#e8f5e9")

        issues_row = ttk.Frame(tab_plan)
        issues_row.pack(fill="both", expand=True, pady=(8, 4))
        ttk.Label(issues_row, text="Проверки:", font=("Segoe UI", 9, "bold")).pack(anchor="w")
        self.issues_text = tk.Text(issues_row, height=9, wrap="word", font=("Consolas", 9))
        self.issues_text.pack(fill="both", expand=True)

        action_row = ttk.Frame(tab_plan)
        action_row.pack(fill="x", pady=6)
        self.allow_reverse = tk.BooleanVar(value=False)
        ttk.Checkbutton(action_row, text="Разрешить коррекцию направления через SERVOx_REVERSED",
                        variable=self.allow_reverse, command=self.rebuild_plan).pack(side="left")
        self.apply_outputs_btn = ttk.Button(action_row, text="Назначить выходы (SERVOx_FUNCTION)",
                                            command=self.on_apply_outputs)
        self.apply_outputs_btn.pack(side="right")
        self.apply_frame_btn = ttk.Button(action_row, text="Записать FRAME_CLASS/FRAME_TYPE",
                                           command=self.on_apply_frame)
        self.apply_frame_btn.pack(side="right", padx=6)
        ttk.Button(action_row, text="Экспорт .params", command=self.on_export_params).pack(side="right", padx=6)
        ttk.Button(action_row, text="Отчёт в файл", command=self.on_export_report).pack(side="right")
        ttk.Button(action_row, text="Пересобрать план", command=self.rebuild_plan).pack(side="right", padx=6)

        self.report_text = tk.Text(tab_plan, height=8, wrap="word", font=("Consolas", 9))
        self.report_text.pack(fill="both", expand=False)

        ttk.Label(tab_plan, text="Проверка параметров выходов и моторов (ArduCopter 4.7.2):",
                  font=("Segoe UI", 9, "bold")).pack(anchor="w", pady=(10, 2))
        self.outputs_text = tk.Text(tab_plan, height=7, wrap="word", font=("Consolas", 9))
        self.outputs_text.pack(fill="both", expand=False)

        # --- вкладка "Журнал" ---
        tab_log = ttk.Frame(right, padding=6)
        right.add(tab_log, text="Журнал")
        self.log_text = tk.Text(tab_log, wrap="word", font=("Consolas", 9))
        scroll = ttk.Scrollbar(tab_log, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.log_text.pack(fill="both", expand=True)

        # --- низ: тест мотора и стоп ---
        bottom = ttk.Frame(self, padding=(8, 4))
        bottom.pack(side="bottom", fill="x")
        self.stop_btn = ttk.Button(bottom, text="СТОП — остановить тест мотора",
                                   command=self.on_stop_test)
        self.stop_btn.pack(side="right")
        ttk.Label(bottom, text="Предусловия ArduPilot для теста: аппарат на земле, "
                               "safety switch в положении Safe, RC откалиброван, "
                               "двигатели disarm.").pack(side="left")

        self._fill_expected()
        self.canvas.redraw()

    # ---------------- журнал ----------------
    def log(self, text):
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{stamp}] {text}\n")
        self.log_text.see("end")
        self.queue.put(("log", text))

    def _poll(self):
        try:
            while True:
                kind, payload = self.queue.get_nowait()
                if kind == "log":
                    pass
                elif kind == "status":
                    self.status_label.configure(text=payload)
                elif kind == "info":
                    self.info_label.configure(text=payload)
                elif kind == "connected":
                    self._on_connected(payload)
                elif kind == "params":
                    self._on_params(payload)
                elif kind == "ack":
                    self._on_ack(payload)
                elif kind == "report":
                    self._render_report(payload)
                elif kind == "error":
                    messagebox.showerror("Ошибка", payload)
                elif kind == "plan":
                    self._render_plan(payload)
        except queue.Empty:
            pass
        self.after(100, self._poll)

    def _set_status(self, text, ok=True):
        self.status_label.configure(text=text, foreground="#080" if ok else "#a00")

    # ---------------- фоновые операции ----------------
    def _run(self, func):
        if self.busy:
            self.log("Занято — дождитесь окончания текущей операции.")
            return
        self.busy = True
        self._set_status("Операция выполняется...", ok=False)

        def worker():
            try:
                func()
            except Exception as exc:  # noqa: BLE001
                self.queue.put(("error", f"{exc}"))
                self.queue.put(("status", "Ошибка"))
            finally:
                self.busy = False

        threading.Thread(target=worker, daemon=True).start()

    def on_connect(self):
        port = self.port_box.get().strip()
        if not port:
            self.log("Порт не выбран.")
            return
        baud = int(self.baud_box.get().strip() or 115200)

        def work():
            link = MAVLinkLink(port, baud)
            link.on_status = lambda sev, txt: self.queue.put(("log", f"[AP] {txt}"))
            with self.link_lock:
                link.open()
                hb = link.wait_heartbeat(12.0)
                if hb is None:
                    link.close()
                    raise RuntimeError("Heartbeat не получен: это не MAVLink-устройство или "
                                       "неверны порт/скорость.")
                self.link = link
            self.queue.put(("connected", hb))

        self._run(work)

    def on_disconnect(self):
        if self.link is not None:
            try:
                self.link.motor_test_stop()
            except Exception:
                pass
            self.link.close()
            self.link = None
        self._set_status("Нет соединения", ok=False)
        self.info_label.configure(text="")

    def on_refresh_ports(self):
        ports = [p for p, _ in list_ports()]
        self.port_box.configure(values=ports)
        self.log(f"Порты: {', '.join(ports) if ports else 'не найдены'}")

    def _on_connected(self, hb):
        ap = hb["autopilot"]
        name = {MAV_AUTOPILOT_ARDUPILOTMEGA: "ArduPilot", MAV_AUTOPILOT_PX4: "PX4"}.get(ap, f"код {ap}")
        self._set_status(f"Подключено: {name}", ok=True)
        self.log(f"Heartbeat: sys={self.link.system_id} comp={self.link.component_id} "
                 f"autopilot={name} armed={hb['armed']}")
        if ap != MAV_AUTOPILOT_ARDUPILOTMEGA:
            self.queue.put(("error", "Плата отвечает не как ArduPilot. Это приложение "
                                     "настраивает маппинг моторов ArduCopter."))
        self.after(300, self.on_read_params)
        self.after(600, self.on_request_version)

    def on_request_version(self):
        """Запросить версию прошивки (CMD 520 -> MSG_AUTOPILOT_VERSION, §5.3)."""
        if self.link is None:
            return

        def work():
            with self.link_lock:
                info = self.link.request_autopilot_version(timeout=6.0)
            self.queue.put(("ack", ("version", info)))

        self._run(work)

    def on_read_params(self):
        if self.link is None:
            self.log("Нет соединения.")
            return

        def work():
            with self.link_lock:
                params = self.link.request_param_list(40.0)
            self.queue.put(("params", params))

        self._run(work)

    def _on_params(self, params):
        self.params = params
        self.log(f"Прочитано параметров: {len(params)}")
        self._load_frame_params()
        self._load_servo_params()

    def _param(self, name, default=None):
        item = self.params.get(name)
        return item[0] if item else default

    def _load_frame_params(self):
        fc = self._param("FRAME_CLASS")
        ft = self._param("FRAME_TYPE")
        if fc is not None and fc in FRAME_CLASS:
            self.frame_class = int(fc)
            self.class_box.current(self.class_index(self.frame_class))
        if ft is not None and ft in FRAME_TYPE:
            self.frame_type = int(ft)
            self.type_box.current(self.type_index(self.frame_type))
        self._fill_expected()
        self.canvas.redraw()
        self.log(f"На плате: FRAME_CLASS={self.frame_class} ({FRAME_CLASS[self.frame_class]}), "
                 f"FRAME_TYPE={self.frame_type} ({FRAME_TYPE.get(self.frame_type, '?')})")

    @staticmethod
    def class_index(value):
        values = sorted(FRAME_CLASS)
        return values.index(value) if value in values else 0

    @staticmethod
    def type_index(value):
        values = sorted(FRAME_TYPE)
        return values.index(value) if value in values else 0

    def _load_servo_params(self):
        self.current_function = {}
        self.current_reversed = {}
        for output in range(1, 17):
            func = self._param(f"SERVO{output}_FUNCTION")
            if func is not None:
                self.current_function[output] = int(func)
            rev = self._param(f"SERVO{output}_REVERSED")
            if rev is not None:
                self.current_reversed[output] = int(rev)
        self._render_plan_table()
        self.rebuild_plan()

    def _fill_expected(self):
        self.expected_tree.delete(*self.expected_tree.get_children())
        try:
            motors = attach_physical_outputs(frame_motors(self.frame_class, self.frame_type),
                                            self.current_function)
        except KeyError as exc:
            self.expected_tree.insert("", "end", values=("-", str(exc), "", "", "", "", "", ""))
            self._render_frame_profile([])
            return
        for index, motor in enumerate(motors):
            self.expected_tree.insert("", "end", values=(
                f"M{motor.motor_number}", motor.position_ru, motor.layer_ru,
                motor.direction_ru, motor.test_order, motor.test_button,
                motor.servo_function_id, motor.physical_output if motor.physical_output else "—"),
                tags=("odd",) if index % 2 else ())
        self._render_frame_profile(motors)
        self._fill_checklist(motors)

    def _render_frame_profile(self, motors):
        """Подпись профиля и предупреждения о границах профиля (§5.1, §8)."""
        self.profile_label.configure(
            text=f"Профиль: {PROFILE_ID} — {PROFILE_NAME}"
                 if (self.frame_class, self.frame_type) == (PROFILE_FRAME_CLASS, PROFILE_FRAME_TYPE)
                 else f"Выбрана рама {FRAME_CLASS.get(self.frame_class, self.frame_class)}/"
                      f"{FRAME_TYPE.get(self.frame_type, self.frame_type)} — это НЕ профиль "
                      f"{PROFILE_ID}"
        )
        lines = []
        if (self.frame_class, self.frame_type) != (PROFILE_FRAME_CLASS, PROFILE_FRAME_TYPE):
            lines.append(
                "WARN: профиль приложения — OCTO QUAD X (X8). Для выбранной комбинации "
                "класса/типа раскладка X8 не применяется: показана таблица, "
                "соответствующая выбранному типу в исходниках ArduPilot 4.7.2."
            )
        if self.frame_class == 3:
            lines.append("WARN: FRAME_CLASS=3 (OCTA) — 8 независимых лучей, не коаксиальные пары.")
        if self.frame_type in (0, 12, 14):
            names = {0: "PLUS", 12: "BF_X (Betaflight X)", 14: "CW_X (Clockwise X)"}
            lines.append(f"WARN: FRAME_TYPE={self.frame_type} ({names[self.frame_type]}) — "
                         "это не обычный X8; убедитесь в схеме своей рамы.")
        if self.frame_type in (20, 21):
            lines.append("WARN: co-rotating вариант (X_COR/CW_X_COR). Для co-rotating X8 "
                         "документация указывает FRAME_CLASS=17 со скриптом X8-corotating.lua — "
                         "это отдельный режим, автоматически он не включается.")
        if self.frame_class == 17:
            lines.append("WARN: FRAME_CLASS=17 (DYNAMIC_SCRIPTING_MATRIX) требует скрипта "
                         "Lua — этот профиль не покрывает такой режим.")
        self.frame_warning.delete("1.0", "end")
        self.frame_warning.insert("1.0", "\n".join(lines) if lines else
                                   "Профиль соответствует обычному OCTO QUAD X (X8).")

    def _fill_checklist(self, motors):
        """Чек-лист проверки вращения: 8 строк, прогресс X/8 (§5.7)."""
        self.checklist_tree.delete(*self.checklist_tree.get_children())
        for motor in motors:
            result = self.checklist.get(motor.motor_number, "не проверено")
            tag = ()
            if result == "совпало":
                tag = ("ok",)
            elif result == "не совпало":
                tag = ("bad",)
            self.checklist_tree.insert("", "end", values=(
                f"M{motor.motor_number}", motor.position_ru, motor.layer_ru,
                motor.direction_ru, result), tags=tag)
        checked = sum(1 for value in self.checklist.values() if value != "не проверено")
        total = len(motors)
        self.checklist_progress.configure(text=f"Проверено {checked}/{total}")

    def on_frame_change(self, _event=None):
        self.frame_class = sorted(FRAME_CLASS)[self.class_box.current()]
        self.frame_type = sorted(FRAME_TYPE)[self.type_box.current()]
        self._fill_expected()
        self.canvas.redraw()
        self.rebuild_plan()
        self.log(f"Выбрана рама: FRAME_CLASS={self.frame_class} ({FRAME_CLASS[self.frame_class]}), "
                 f"FRAME_TYPE={self.frame_type} ({FRAME_TYPE.get(self.frame_type, '?')})")

    # ---------------- идентификация ----------------
    def on_motor_test(self):
        if self.link is None:
            self.log("Нет соединения.")
            return
        if not self.spin_ok.get():
            messagebox.showwarning("Безопасность", "Подтвердите, что пропеллеры сняты.")
            return
        motor = int(self.motor_box.get().split("M")[1])
        percent = float(self.throttle.get() or 10)
        timeout = 3.0
        # ArduPilot ищет мотор по номеру ТЕСТА, а не по номеру мотора
        # (AP_MotorsMatrix::_output_test_seq -> _test_order[i] == motor_seq).
        test_seq = test_order_for_motor(self.frame_class, self.frame_type, motor)
        if test_seq is None:
            messagebox.showerror("Тест мотора",
                                 f"Для рамы {FRAME_CLASS.get(self.frame_class)}/"
                                 f"{FRAME_TYPE.get(self.frame_type)} нет мотора M{motor}.")
            return
        self._start_motor_test(motor, test_seq, percent, timeout)

    def on_button_test(self, letter: str):
        """Motor Test по кнопке A…H (порядок теста ArduPilot, §5.2)."""
        if self.link is None:
            self.log("Нет соединения.")
            return
        if not self.spin_ok.get():
            messagebox.showwarning("Безопасность", "Подтвердите, что пропеллеры сняты.")
            return
        index = "ABCDEFGH".index(letter) + 1
        try:
            motors = frame_motors(self.frame_class, self.frame_type)
        except KeyError:
            return
        target = next((m for m in motors if m.test_order == index), None)
        if target is None:
            messagebox.showerror("Тест мотора",
                                 f"В раме {FRAME_CLASS.get(self.frame_class)}/"
                                 f"{FRAME_TYPE.get(self.frame_type)} нет мотора с номером теста {index}.")
            return
        percent = float(self.throttle.get() or 10)
        self._start_motor_test(target.motor_number, index, percent, 3.0)

    def _start_motor_test(self, motor: int, test_seq: int, percent: float, timeout: float):
        """Общая отправка теста: снимок armed, проверки, сериализация (§5.6, §6)."""
        # Состояние ARMED блокирует обычный сценарий настройки (§6).
        # Снимок берём ДО отправки команды: сам тест поднимает armed на плате.
        if self.link is not None and self.link.armed:
            messagebox.showerror(
                "Тест мотора",
                "Аппарат находится в состоянии ARMED — тест заблокирован. "
                "Снимите armed на плате и повторите.")
            self.log("Тест заблокирован: аппарат ARMED.")
            return
        self.armed_before_test = bool(self.link.armed) if self.link else False
        self.test_running = True

        def work():
            with self.link_lock:
                ack = self.link.motor_test(test_seq, percent, timeout)
            self.queue.put(("ack", ("motor_test", motor, test_seq, ack)))

        self._run(work)

    def on_stop_test(self):
        if self.link is None:
            return
        try:
            with self.link_lock:
                self.link.motor_test_stop()
            self.test_running = False
            self.log("Тест мотора остановлен.")
        except Exception as exc:  # noqa: BLE001
            self.log(f"Ошибка остановки: {exc}")

    def _on_ack(self, payload):
        kind = payload[0]
        if kind == "motor_test":
            _tag, motor, test_seq, ack = payload
            info = self._motor_info(motor)
            place = f"{info['position']}, {info['level']}" if info else "?"
            if ack is None:
                self.log(f"Тест M{motor} (тест №{test_seq}, кнопка {info['button'] if info else '?'}): "
                         "нет ACK от платы.")
            elif ack["result"] == 0:
                self.log(f"Тест M{motor} (тест №{test_seq}, кнопка {info['button'] if info else '?'}), "
                         f"ожидаемое место: {place}. ACK ACCEPTED, мотор крутится "
                         f"{self.throttle.get()}%. ACK — не подтверждение вращения: "
                         "отметьте фактический мотор и направление на виде сверху.")
            else:
                self.log(f"Тест M{motor} (тест №{test_seq}): ACK result={ack['result']} — плата "
                         "отклонила команду. Проверьте: аппарат на земле, safety switch в Safe, "
                         "RC откалиброван, аппарат disarm.")
        elif kind == "version":
            _tag, info = payload
            self.firmware_version = info
            text = info.get("version_text", "?") if info else "?"
            if info and info.get("version_major") == 4 and info.get("version_minor") == 7:
                self.log(f"Прошивка: {text} (линейка {SUPPORTED_FIRMWARE_SERIES} — профиль применим).")
            else:
                self.log(f"Прошивка: {text}. Ожидалась линейка {SUPPORTED_FIRMWARE_SERIES}.x — "
                         "проверьте, что таблица раскладки соответствует вашей прошивке.")

    def _motor_info(self, motor: int) -> Optional[dict]:
        try:
            motors = frame_motors(self.frame_class, self.frame_type)
        except KeyError:
            return None
        for item in motors:
            if item.motor_number == motor:
                return {
                    "position": item.position_ru,
                    "level": item.layer_ru,
                    "button": item.test_button,
                    "order": item.test_order,
                    "direction": item.direction_ru,
                }
        return None

    def on_save_checklist(self):
        """Записать результат ручной проверки вращения для выбранного мотора (§5.7)."""
        selection = self.checklist_tree.selection()
        if not selection:
            messagebox.showinfo("Чек-лист", "Выберите мотор в таблице чек-листа.")
            return
        item = self.checklist_tree.item(selection[0], "values")
        motor = int(item[0].split("M")[1])
        self.checklist[motor] = self.checklist_result.get()
        self._fill_checklist(frame_motors(self.frame_class, self.frame_type))
        self.log(f"Чек-лист: M{motor} — {self.checklist[motor]}.")

    def on_slot_clicked(self, position, level):
        self.selected_slot = (position, level)
        self.canvas.redraw()
        self.log(f"Выбран слот: {POSITION_RU.get(position, position)}, {LEVEL_RU.get(level, level)}. "
                 "Задайте направление и нажмите «Сохранить наблюдение».")

    def on_save_observation(self):
        if self.selected_slot is None:
            messagebox.showinfo("Наблюдение", "Сначала кликните по кружку мотора на виде сверху.")
            return
        motor = int(self.motor_box.get().split("M")[1])
        position, level = self.selected_slot
        direction = "CCW" if self.direction.get().startswith("CCW") else "CW"
        # слот может занимать только один мотор
        for other_motor, (pos, lvl, _d) in list(self.observed_by_motor.items()):
            if (pos, lvl) == (position, level):
                del self.observed_by_motor[other_motor]
        self.observed_by_motor[motor] = (position, level, direction)
        self.slot_to_motor[(position, level)] = motor
        self.canvas.redraw()
        self._fill_obs_tree()
        self.rebuild_plan()
        self.log(f"Наблюдение: M{motor} -> {POSITION_RU.get(position, position)}, "
                 f"{LEVEL_RU.get(level, level)}, {DIRECTION_RU.get(direction, direction)}")

    def output_of_motor(self, motor):
        """Выход MAIN, на котором сейчас сидит логический мотор."""
        for output, function in self.current_function.items():
            if motor_for_servo_function(function) == motor:
                return output
        return 0

    def _fill_obs_tree(self):
        self.obs_tree.delete(*self.obs_tree.get_children())
        for motor in sorted(self.observed_by_motor):
            position, level, direction = self.observed_by_motor[motor]
            output = self.output_of_motor(motor)
            self.obs_tree.insert("", "end", values=(
                f"M{motor}", output or "-", POSITION_RU.get(position, position),
                LEVEL_RU.get(level, level), DIRECTION_RU.get(direction, direction)))

    def on_clear_observations(self):
        self.observed_by_motor.clear()
        self.slot_to_motor.clear()
        self.canvas.redraw()
        self._fill_obs_tree()
        self.rebuild_plan()
        self.log("Наблюдения очищены.")

    # ---------------- план ----------------
    def _frame_actions(self):
        """Изменения класса и типа рамы, если они на плате не совпадают с выбранными."""
        actions = []
        cur_class = self._param("FRAME_CLASS")
        cur_type = self._param("FRAME_TYPE")
        if cur_class is not None and int(cur_class) != self.frame_class:
            actions.append((
                "FRAME_CLASS", int(cur_class), self.frame_class,
                f"класс рамы {FRAME_CLASS.get(self.frame_class, self.frame_class)} "
                f"(требуется перезагрузка платы)",
            ))
        if cur_type is not None and int(cur_type) != self.frame_type:
            actions.append((
                "FRAME_TYPE", int(cur_type), self.frame_type,
                f"тип рамы {FRAME_TYPE.get(self.frame_type, self.frame_type)} "
                f"(требуется перезагрузка платы)",
            ))
        return actions

    def rebuild_plan(self):
        if not self.current_function:
            self.plan = None
            self.issues_text.delete("1.0", "end")
            self.issues_text.insert("1.0", "Сначала подключитесь к плате и прочитайте параметры.")
            return
        self.plan = build_plan(self.frame_class, self.frame_type, self.observed_by_motor,
                               self.current_function, allow_reverse=self.allow_reverse.get())
        # Диагностика назначений функций моторов (§5.5, §7): отсутствующие и повторные.
        try:
            motor_count = len(frame_motors(self.frame_class, self.frame_type))
        except KeyError:
            motor_count = 8
        self.plan.issues.extend(audit_motor_functions(self.current_function, motor_count))
        frame_actions = self._frame_actions()
        if frame_actions:
            for name, old, new, reason in frame_actions:
                self.plan.issues.append(Issue("WARN", f"На плате {name}={old}, "
                                                      f"в приложении выбрано {new}. {reason}"))
        self._render_plan(self.plan)

    def _check_output_params(self):
        """Проверки параметров моторов и выходов (не блокируют применение плана)."""
        lines = []
        mot_min = self._param("MOT_PWM_MIN")
        mot_max = self._param("MOT_PWM_MAX")
        hover = self._param("MOT_PWM_HOVER")
        trim = self._param("MOT_PWM_TRIM")
        if mot_min is not None and mot_max is not None:
            if mot_min >= mot_max:
                lines.append(f"ERROR: MOT_PWM_MIN={mot_min} >= MOT_PWM_MAX={mot_max} — "
                             "диапазон газа некорректен, моторы работать не будут.")
            else:
                lines.append(f"OK: MOT_PWM_MIN={mot_min} < MOT_PWM_MAX={mot_max} "
                             f"(ArduPilot сам отдаёт мотору это значение).")
        else:
            lines.append("MOT_PWM_MIN/MOT_PWM_MAX не найдены в списке параметров.")
        for name, value in (("MOT_PWM_TRIM", trim), ("MOT_PWM_HOVER", hover)):
            if value is None:
                continue
            if mot_min is not None and mot_max is not None and not (mot_min <= value <= mot_max):
                lines.append(f"WARN: {name}={value} вне диапазона "
                             f"[{mot_min}..{mot_max}].")
            else:
                lines.append(f"OK: {name}={value}")
        for output in sorted(self.current_function):
            if motor_for_servo_function(self.current_function[output]) is None:
                continue
            smin = self._param(f"SERVO{output}_MIN")
            smax = self._param(f"SERVO{output}_MAX")
            strim = self._param(f"SERVO{output}_TRIM")
            rev = self._param(f"SERVO{output}_REVERSED")
            proto = self._param(f"SERVO{output}_PROTOCOL")
            if None not in (smin, smax, strim):
                if smin > smax:
                    lines.append(f"ERROR: SERVO{output}_MIN={smin} > SERVO{output}_MAX={smax}.")
                elif not (smin <= strim <= smax):
                    lines.append(f"WARN: SERVO{output}_TRIM={strim} вне [{smin}..{smax}].")
                else:
                    lines.append(f"OK: выход {output}: MIN={smin} TRIM={strim} MAX={smax}")
            if rev:
                lines.append(f"INFO: выход {output} включён в обратную сторону (REVERSED={rev}).")
            if proto is not None:
                lines.append(f"INFO: выход {output}: протокол ESC SERVO{output}_PROTOCOL={proto} "
                             "(при DShot значения MIN/MAX/TRIM не используются).")
        return lines

    def _render_output_checks(self):
        lines = self._check_output_params()
        self.outputs_text.delete("1.0", "end")
        self.outputs_text.insert("1.0", "\n".join(lines) if lines else "Нет данных.")

    def _render_plan(self, plan):
        self.plan = plan
        self.issues_text.delete("1.0", "end")
        if not plan.issues:
            self.issues_text.insert("1.0", "Замечаний нет.")
        for issue in plan.issues:
            self.issues_text.insert("end", f"{issue.severity}: {issue.text}\n\n")
        self._render_plan_table()
        self._render_output_checks()

    def _render_plan_table(self):
        self.plan_tree.delete(*self.plan_tree.get_children())
        plan_map = self.plan.function_plan if self.plan else {}
        reverse_map = self.plan.reverse_plan if self.plan else {}
        for output in sorted(self.current_function):
            cur = self.current_function.get(output, 0)
            planned = plan_map.get(output, cur)
            motor = motor_for_servo_function(planned)
            phys = ""
            current_motor = motor_for_servo_function(cur)
            if current_motor and current_motor in self.observed_by_motor:
                position, level, direction = self.observed_by_motor[current_motor]
                phys = (f"{POSITION_RU.get(position, position)}, {LEVEL_RU.get(level, level)}, "
                        f"{DIRECTION_RU.get(direction, direction)}")
            cur_rev = self.current_reversed.get(output, 0)
            new_rev = reverse_map.get(output, cur_rev)
            rev_text = f"{cur_rev} -> {new_rev}" if new_rev != cur_rev else ""
            self.plan_tree.insert("", "end", values=(
                output, cur, planned, f"M{motor}" if motor else "-", rev_text, phys),
                tags=("change",) if (planned != cur or rev_text) else ("same",))

    def on_export_params(self):
        if not self.params:
            messagebox.showinfo("Экспорт", "Сначала прочитайте параметры с платы.")
            return
        os.makedirs(CONFIG_DIR, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        path = os.path.join(CONFIG_DIR, f"ardupilot_params_{stamp}.params")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(f"# ArduCopter параметры, выгружено {datetime.now().isoformat(timespec='seconds')}\n")
            fh.write(f"# порт {self.port_box.get()} {self.baud_box.get()}\n")
            for name, (value, ptype) in sorted(self.params.items()):
                fh.write(f"{name}\t{value}\t{ptype}\n")
        self.log(f"Параметры сохранены: {path}")

    def on_export_report(self):
        if not self.report_rows:
            messagebox.showinfo("Отчёт", "Сначала примените план — отчёт появится после записи.")
            return
        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        path = os.path.join(LOG_DIR, f"motor-mapping-{stamp}.md")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(f"# Отчёт по маппингу моторов — {datetime.now().isoformat(timespec='seconds')}\n\n")
            fh.write("| Параметр | Было | Стало | Основание | Проверка |\n|---|---|---|---|---|\n")
            for row in self.report_rows:
                fh.write("| " + " | ".join(str(x) for x in row) + " |\n")
        self.log(f"Отчёт сохранён: {path}")

    # ---------------- применение ----------------
    def on_apply_outputs(self):
        """Назначение выходов — отдельное действие (§5.5)."""
        if self.link is None:
            messagebox.showinfo("Применение", "Нет соединения с платой.")
            return
        if self.plan is None or not self.plan.function_plan:
            messagebox.showinfo("Применение", "Нет плана к применению.")
            return
        if self.plan.errors:
            messagebox.showerror("Применение", "В плане есть ошибки — сначала устраните их:\n\n"
                               + "\n".join(str(i) for i in self.plan.errors))
            return
        actions = self.plan.actions(self.current_function, self.current_reversed)
        if not actions:
            messagebox.showinfo("Применение", "План пуст: всё уже назначено правильно.")
            return
        text = "Будут записаны параметры выходов:\n\n" + "\n".join(
            f"  {name}: {old} -> {new}  ({reason})" for name, old, new, reason in actions)
        if not messagebox.askyesno("Подтверждение записи", text + "\n\nЗаписать на плату?"):
            return

        def work():
            self._apply_actions(actions)

        self._run(work)

    def on_apply_frame(self):
        """Запись FRAME_CLASS/FRAME_TYPE с перезагрузкой и повторной сверкой (§5.4)."""
        if self.link is None:
            messagebox.showinfo("Применение", "Нет соединения с платой.")
            return
        actions = self._frame_actions()
        if not actions:
            messagebox.showinfo("Применение", "На плате уже выбрана та же рама "
                                              "(FRAME_CLASS/FRAME_TYPE совпадают).")
            return
        text = ("Будут записаны параметры рамы:\n\n" + "\n".join(
            f"  {name}: {old} -> {new}  ({reason})" for name, old, new, reason in actions)
            + "\n\nПосле записи потребуется перезагрузка платы — она применит новую раскладку "
              "миксера. Применение займёт около 15 секунд.")
        if not messagebox.askyesno("Подтверждение записи и перезагрузки", text
                                   + "\n\nЗаписать и перезагрузить плату?"):
            return

        def work():
            self._apply_frame_with_reboot(actions)

        self._run(work)

    def _apply_frame_with_reboot(self, actions):
        """Записать FRAME_*, дождаться disarm, перезагрузить, перечитать и сверить."""
        link = self.link
        os.makedirs(CONFIG_DIR, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        backup_path = os.path.join(CONFIG_DIR, f"ardupilot_backup_{stamp}.params")
        with self.link_lock:
            snapshot = link.request_param_list(40.0)
        with open(backup_path, "w", encoding="utf-8") as fh:
            fh.write(f"# Бэкап параметров перед изменением рамы "
                     f"{datetime.now().isoformat(timespec='seconds')}\n")
            for name, (value, ptype) in sorted(snapshot.items()):
                fh.write(f"{name}\t{value}\t{ptype}\n")
        self.queue.put(("log", f"Бэкап параметров: {backup_path}"))

        rows = []
        for name, old, new, reason in actions:
            with self.link_lock:
                echo = link.set_param(name, new, timeout=6.0)
            if echo is None:
                ok = "НЕТ ЭХА"
            elif abs(float(echo[0]) - float(new)) < 1e-6:
                ok = f"OK (прочитано {echo[0]})"
            else:
                ok = f"РАСХОЖДЕНИЕ (прочитано {echo[0]})"
            rows.append((name, old, new, reason, ok))
            self.queue.put(("log", f"{name}: {old} -> {new} :: {ok}"))

        # Перед перезагрузкой: остановить тест мотора и дождаться disarm.
        # Тест мотора поднимает soft_armed (motor_test.cpp:169), а перезагрузка
        # при armed отклоняется (GCS_Common.cpp:3643-3650).
        if self.test_running:
            with self.link_lock:
                link.motor_test_stop()
            self.test_running = False
            self.queue.put(("log", "Тест мотора остановлен перед перезагрузкой."))
        with self.link_lock:
            disarmed = link.wait_disarmed(15.0)
        if not disarmed:
            self.queue.put(("log", "WARNING: аппарат не подтвердил disarm — перезагрузка "
                                    "может быть отклонена."))
            return

        self.queue.put(("log", "Перезагрузка платы..."))
        with self.link_lock:
            ack = link.reboot()
        if ack is None:
            self.queue.put(("log", "Перезагрузка: нет ACK от платы."))
            return
        self.queue.put(("log", f"Перезагрузка: ACK result={ack['result']}."))

        # Плата перезагружается сразу после ACK — ждём heartbeat и перечитываем.
        self._wait_for_reboot_and_verify(actions, rows)

    def _wait_for_reboot_and_verify(self, actions, rows):
        """Дождаться платы после перезагрузки, перечитать FRAME_* и сверить (§5.4)."""
        link = self.link
        deadline = time.monotonic() + 90.0
        heartbeat_seen = False
        while time.monotonic() < deadline:
            time.sleep(0.5)
            try:
                with self.link_lock:
                    frames = link.read_frames(0.5)
            except Exception as exc:  # noqa: BLE001
                self.queue.put(("log", f"Связь во время перезагрузки: {exc}"))
                continue
            for _v, _s, _c, msgid, _payload in frames:
                if msgid == 0:  # MSG_HEARTBEAT
                    heartbeat_seen = True
            if heartbeat_seen:
                break

        if not heartbeat_seen:
            self.queue.put(("log", "После перезагрузки heartbeat не получен — проверьте "
                                    "связь и перечитайте параметры вручную."))
            self.report_rows = rows
            self.queue.put(("report", rows))
            return

        self.queue.put(("log", "Плата вернулась, повторно читаю FRAME_CLASS/FRAME_TYPE..."))
        verified = []
        for name, old, new, reason in actions:
            value = None
            deadline_param = time.monotonic() + 15.0
            while time.monotonic() < deadline_param and value is None:
                try:
                    with self.link_lock:
                        item = link.read_param(name, timeout=3.0)
                except Exception:  # noqa: BLE001
                    item = None
                if item:
                    value = item[0]
                else:
                    time.sleep(0.5)
            if value is None:
                ok = "НЕ ПРОВЕРЕНО (нет ответа)"
            elif abs(float(value) - float(new)) < 1e-6:
                ok = f"OK после перезагрузки (прочитано {value})"
            else:
                ok = f"РАСХОЖДЕНИЕ после перезагрузки (прочитано {value})"
            verified.append((name, old, new, reason, ok))
            self.queue.put(("log", f"{name}: проверка после перезагрузки -> {ok}"))

        self.report_rows = verified
        self.queue.put(("report", verified))
        self.queue.put(("log", "Перезагрузка завершена, значения сверены."))

    def _apply_actions(self, actions):
        link = self.link
        os.makedirs(CONFIG_DIR, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        backup_path = os.path.join(CONFIG_DIR, f"ardupilot_backup_{stamp}.params")
        with self.link_lock:
            snapshot = link.request_param_list(40.0)
        with open(backup_path, "w", encoding="utf-8") as fh:
            fh.write(f"# Бэкап параметров перед изменением маппинга "
                     f"{datetime.now().isoformat(timespec='seconds')}\n")
            for name, (value, ptype) in sorted(snapshot.items()):
                fh.write(f"{name}\t{value}\t{ptype}\n")
        self.queue.put(("log", f"Бэкап параметров: {backup_path}"))

        rows = []
        for name, old, new, reason in actions:
            with self.link_lock:
                echo = link.set_param(name, new, timeout=6.0)
            if echo is None:
                ok = "НЕТ ЭХА"
            elif abs(float(echo[0]) - float(new)) < 1e-6:
                ok = f"OK (прочитано {echo[0]})"
            else:
                ok = f"РАСХОЖДЕНИЕ (прочитано {echo[0]})"
            rows.append((name, old, new, reason, ok))
            self.queue.put(("log", f"{name}: {old} -> {new} :: {ok}"))
        self.report_rows = rows
        self.queue.put(("report", rows))
        self.queue.put(("log", "Запись завершена. Требуется перезагрузка платы для применения "
                               "нового маппинга (FRAME_CLASS/FRAME_TYPE и функции выходов)."))

    def _render_report(self, rows):
        self.report_text.delete("1.0", "end")
        self.report_text.insert("1.0", "Параметр | было | стало | основание | проверка\n")
        for row in rows:
            self.report_text.insert("end", " | ".join(str(x) for x in row) + "\n")

    # ---------------- закрытие ----------------
    def _on_close(self):
        try:
            if self.link is not None:
                self.link.motor_test_stop()
                self.link.close()
        except Exception:
            pass
        self.destroy()


def main():
    app = App()
    if "--selftest" in sys.argv:
        app.update()
        app.update_idletasks()
        app.destroy()
        print("GUI: построение окна прошло без ошибок.")
        return 0
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
