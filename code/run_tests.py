"""Запуск всех тестов проекта.

Запуск: python run_tests.py
"""
from __future__ import annotations

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

TESTS = [
    "test_motor_mapping_core.py",
    "test_mavlink_codec.py",
    "test_motor_configurator_gui.py",
]


def main():
    env = dict(os.environ)
    env["PYTHONIOENCODING"] = "utf-8"
    failed = []
    for name in TESTS:
        print(f"\n=== {name} ===")
        result = subprocess.run([sys.executable, os.path.join(HERE, name)],
                                cwd=HERE, env=env, capture_output=True, text=True,
                                encoding="utf-8", errors="replace")
        print(result.stdout.strip())
        if result.stderr.strip():
            print(result.stderr.strip())
        if result.returncode != 0:
            failed.append(name)
    print("\n" + "=" * 60)
    if failed:
        print(f"ПРОВАЛЕНЫ тесты: {', '.join(failed)}")
        return 1
    print(f"Все наборы тестов пройдены ({len(TESTS)} шт.).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
