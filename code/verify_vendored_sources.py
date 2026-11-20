#!/usr/bin/env python3
"""Проверка, что вендоренные файлы ArduPilot побайтово совпадают с upstream.

Назначение: `research/ardupilot/` содержит выдержки из исходников ArduPilot
4.7.2 (GPLv3), полученные из закреплённого коммита. Чтобы утверждение
«файл взят без изменений» оставалось проверяемым, этот скрипт скачивает
те же файлы из upstream и сверяет SHA-256.

Запуск:
    python code/verify_vendored_sources.py [--commit <sha>]

Код возврата 0 — все файлы совпали, 1 — есть расхождения или файла нет.
Сеть нужна только для этого скрипта; само приложение работает офлайн.
"""
from __future__ import annotations

import argparse
import hashlib
import sys
import urllib.request
from pathlib import Path

COMMIT = "fb773567896014da007bca58358704d4a0d8516f"
UPSTREAM = "https://raw.githubusercontent.com/ArduPilot/ArduPilot/{commit}/{path}"
VENDOR_DIR = Path(__file__).resolve().parent.parent / "research" / "ardupilot"

# локальное имя (с префиксом пути в "__") -> путь в репозитории ArduPilot
MAPPING = {
    "ArduCopter__Copter.cpp": "ArduCopter/Copter.cpp",
    "ArduCopter__GCS_MAVLink_Copter.cpp": "ArduCopter/GCS_MAVLink_Copter.cpp",
    "ArduCopter__motor_test.cpp": "ArduCopter/motor_test.cpp",
    "ArduCopter__Parameters.cpp": "ArduCopter/Parameters.cpp",
    "libraries__AP_Motors__AP_MotorsMatrix.cpp": "libraries/AP_Motors/AP_MotorsMatrix.cpp",
    "libraries__AP_Motors__AP_MotorsMatrix.h": "libraries/AP_Motors/AP_MotorsMatrix.h",
    "libraries__AP_Motors__AP_Motors_Class.h": "libraries/AP_Motors/AP_Motors_Class.h",
    "libraries__AP_Motors__AP_MotorsCoax.cpp": "libraries/AP_Motors/AP_MotorsCoax.cpp",
    "libraries__GCS_MAVLink__GCS.cpp": "libraries/GCS_MAVLink/GCS.cpp",
    "libraries__GCS_MAVLink__GCS_Common.cpp": "libraries/GCS_MAVLink/GCS_Common.cpp",
    "libraries__GCS_MAVLink__GCS_MAVLink.cpp": "libraries/GCS_MAVLink/GCS_MAVLink.cpp",
    "libraries__GCS_MAVLink__GCS_MAVLink_Parameters.cpp": "libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp",
    "libraries__GCS_MAVLink__GCS_Param.cpp": "libraries/GCS_MAVLink/GCS_Param.cpp",
    "libraries__SRV_Channel__SRV_Channel.cpp": "libraries/SRV_Channel/SRV_Channel.cpp",
    "libraries__SRV_Channel__SRV_Channel.h": "libraries/SRV_Channel/SRV_Channel.h",
}


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def fetch(url: str) -> bytes:
    request = urllib.request.Request(url, headers={"User-Agent": "quav5-motor-configurator-verify/1.0"})
    with urllib.request.urlopen(request, timeout=60) as response:
        return response.read()


def main() -> int:
    parser = argparse.ArgumentParser(description="Сверка вендоренных файлов ArduPilot с upstream")
    parser.add_argument("--commit", default=COMMIT, help="коммит ArduPilot (по умолчанию 4.7.2)")
    args = parser.parse_args()

    if not VENDOR_DIR.is_dir():
        print(f"каталог не найден: {VENDOR_DIR}")
        return 1

    failures = []
    print(f"ArduPilot commit {args.commit}")
    print(f"каталог: {VENDOR_DIR}")
    for local_name, upstream_path in sorted(MAPPING.items()):
        local_path = VENDOR_DIR / local_name
        if not local_path.is_file():
            failures.append(f"{local_name}: локальный файл отсутствует")
            print(f"  FAIL {local_name}: нет локального файла")
            continue
        url = UPSTREAM.format(commit=args.commit, path=upstream_path)
        try:
            remote = fetch(url)
        except Exception as exc:  # noqa: BLE001 - сеть может быть недоступна
            failures.append(f"{local_name}: не скачан ({exc})")
            print(f"  FAIL {local_name}: загрузка не удалась ({exc})")
            continue
        local_hash = sha256_file(local_path)
        remote_hash = sha256_bytes(remote)
        if local_hash == remote_hash:
            print(f"  OK   {local_name}  sha256={local_hash[:16]}")
        else:
            failures.append(f"{local_name}: sha256 {local_hash[:16]} != upstream {remote_hash[:16]}")
            print(f"  FAIL {local_name}: local={local_hash[:16]} upstream={remote_hash[:16]}")

    license_path = VENDOR_DIR / "COPYING.txt"
    if not license_path.is_file():
        failures.append("COPYING.txt: лицензия GPLv3 не приложена")
        print("  FAIL COPYING.txt отсутствует")
    else:
        print(f"  OK   COPYING.txt (GPLv3), sha256={sha256_file(license_path)[:16]}")

    if failures:
        print(f"\nрасхождений: {len(failures)}")
        for item in failures:
            print(f"  - {item}")
        return 1
    print(f"\nвсе {len(MAPPING)} файлов совпадают с upstream")
    return 0


if __name__ == "__main__":
    sys.exit(main())
