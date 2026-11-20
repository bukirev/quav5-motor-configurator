# 2026-11-20 — Публикация наработок на GitHub

## Итог

Проект опубликован в публичный репозиторий **[bukirev/quav5-motor-configurator](https://github.com/bukirev/quav5-motor-configurator)**
(аккаунт `bukirev`, авторизация `gh` — `gho_****`, scopes `gist/read:org/repo/workflow`).
Репозиторий создан с нуля: `git init -b main`, один коммит, push в `origin/main`.

## Решения пользователя (перед публикацией)

1. **Снимок параметров PX4** (`configs/px4_params_COM18_20261026-172517.params`) публикуется
   **без правок**, вместе с идентификаторами чипов реальной платы
   (`CAL_ACC0_ID 3670026`, `CAL_BARO0_ID 3997730`, `CAL_GYRO*_ID`, `CAL_MAG0_ID 396825`).
   Это не секреты и не персональные данные, но факт раскрытия зафиксирован здесь явно.
2. **`research/`** публикуется частично: только файлы, нужные для сверки выводов с исходником
   (AP_MotorsMatrix, motor_test, SRV_Channel, GCS_*.cpp, Parameters.cpp, Copter.cpp),
   плюс `research/NOTICE.md` с атрибуцией. Остальное осталось локально.
3. **Лицензия на собственный код не выбрана** — лицензионный файл в репозиторий не добавлялся,
   в `README.md` добавлен раздел «Лицензия» с честной формулировкой про режим
   «все права защищены».

## Что подготовлено для публикации

| Файл | Назначение |
|---|---|
| `research/NOTICE.md` (новый) | Атрибуция: что от ArduPilot, под какой лицензией, что побайтово, а что производное; перечень того, что не публикуется, и способ вернуть кросс-проверку с pymavlink |
| `research/ardupilot/COPYING.txt` (новый) | Текст GPL-3.0 ArduPilot, извлечён из upstream по коммиту `fb773567…` |
| `code/verify_vendored_sources.py` (новый) | Сверяет SHA-256 публикуемых файлов ArduPilot с upstream: скачивает те же файлы по коммиту и сравнивает |
| `.gitignore` (новый) | Исключает `!!!px4.zip`, `research/px4_src/`, `research/mavlink/`, `research/ardupilot_tag/`, `research/docs_actuators_v1.15.txt`, нецитируемые файлы ArduPilot, `__pycache__` |
| `README.md` (изменён) | Разделы «Запуск», «Публикация на GitHub», «Лицензия»; уточнено описание `research/` |

## Проверка вендоренных исходников

`python code/verify_vendored_sources.py` → **15/15 файлов совпадают с upstream побайтово**
(SHA-256), плюс приложен `COPYING.txt`:

```
ArduCopter__Copter.cpp            ad70b97c197955d0
ArduCopter__GCS_MAVLink_Copter.cpp 7a523c1a6aa68def
ArduCopter__Parameters.cpp         4a4248b8395b16dc
ArduCopter__motor_test.cpp         df946f0fa1324226
libraries__AP_Motors__AP_MotorsCoax.cpp          cda2045db85006e5
libraries__AP_Motors__AP_MotorsMatrix.cpp        54118f5782b699b8
libraries__AP_Motors__AP_MotorsMatrix.h          f8f210d67d187aca
libraries__AP_Motors__AP_Motors_Class.h          13394f23e3a5bdf4
libraries__GCS_MAVLink__GCS.cpp                  551f42adbfe57862
libraries__GCS_MAVLink__GCS_Common.cpp           b660c5f53a99c89b
libraries__GCS_MAVLink__GCS_MAVLink.cpp          9819f92302f16b4d
libraries__GCS_MAVLink__GCS_MAVLink_Parameters.cpp beab49720f851695
libraries__GCS_MAVLink__GCS_Param.cpp            668b245bf81cb9b3
libraries__SRV_Channel__SRV_Channel.cpp          805e8ed6551433f4
libraries__SRV_Channel__SRV_Channel.h            4d9549e031c11523
COPYING.txt                                      8ceb4b9ee5adedde
```

Имена файлов в `research/ardupilot/` переименованы (`путь/к/файлу` → `путь__к__файлу`),
**содержимое не менялось** — это подтверждено сверкой хешей.

## Что осталось локально (в репозиторий не вошло)

| Материал | Причина |
|---|---|
| `research/px4_src/` (24 файла) | сверка велась с ArduPilot; PX4-режим проектом не поддерживается |
| `research/mavlink/` (`common.xml`, `common_crc.xml`, `pymavlink_ref.py`, `mavlink_parse.py`) | эталон для кросс-проверки кодека; при его отсутствии тест даёт SKIP, а не падение |
| `research/ardupilot_tag/` | дубликат подмножества `research/ardupilot/` |
| `research/docs_actuators_v1.15.txt` | справочник PX4 по актуаторам |
| `!!!px4.zip` | архив прежней сессии PX4; 9 из 12 файлов побайтово совпадают с текущими, изменены только `README.md`, `code/README.md`, `docs/README.md` |
| 9 файлов ArduPilot, не цитируемые в `docs/` и `logs/` | сокращение объёма; на выводы не влияют |

Проверка дублирования архива: SHA-256 **9 из 12** файлов `!!!px4.zip` совпал с текущими
файлами репозитория; оставшиеся три (`README.md`, `code/README.md`, `docs/README.md`)
обновлялись позже — содержимое архива не потеряно, оно в репозитории в более свежем виде.

## Состояние кодовой базы на момент публикации

- `python code/run_tests.py` — 3 набора, все проверки проходят.
- `python code/motor_configurator.py --selftest` — GUI собирается без ошибок.
- 10 раскладок рам сверены с `AP_MotorsMatrix.cpp` программно.

## Что осталось непроверенным

- **Плата не подключена**: приём `PARAM_SET`, `MAV_CMD_DO_MOTOR_TEST`, CMD 520/246,
  фактические `SERVOx_FUNCTION`, разводка и направление вращения на железе не проверены.
- Состояние подписки и реальное наличие DroneCAN-узлов на этой плате не подтверждены.
- Лицензия проекта не выбрана — до её выбора публикация не даёт прав на использование кода.
