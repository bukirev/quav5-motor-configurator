# NOTICE — сторонние материалы в `research/`

Каталог `research/` — это **справочные материалы**, использованные при исследовании
маппинга моторов ArduPilot 4.7.2. Они не являются частью приложения
(`code/`) и не покрываются лицензией самого проекта (на момент публикации
лицензия проекта не выбрана — см. раздел «Лицензия» в корневом `README.md`).

Каждый сторонний файл распространяется под лицензией его автора. Ниже —
перечень, кто что принёс и под чем.

## Опубликованные исходники ArduPilot (GPL-3.0)

| Файл в `research/ardupilot/` | Путь в upstream | Лицензия |
|---|---|---|
| `ArduCopter__Copter.cpp` | `ArduCopter/Copter.cpp` | GPL-3.0 |
| `ArduCopter__GCS_MAVLink_Copter.cpp` | `ArduCopter/GCS_MAVLink_Copter.cpp` | GPL-3.0 |
| `ArduCopter__motor_test.cpp` | `ArduCopter/motor_test.cpp` | GPL-3.0 |
| `ArduCopter__Parameters.cpp` | `ArduCopter/Parameters.cpp` | GPL-3.0 |
| `libraries__AP_Motors__AP_MotorsMatrix.cpp` | `libraries/AP_Motors/AP_MotorsMatrix.cpp` | GPL-3.0 |
| `libraries__AP_Motors__AP_MotorsMatrix.h` | `libraries/AP_Motors/AP_MotorsMatrix.h` | GPL-3.0 |
| `libraries__AP_Motors__AP_Motors_Class.h` | `libraries/AP_Motors/AP_Motors_Class.h` | GPL-3.0 |
| `libraries__AP_Motors__AP_MotorsCoax.cpp` | `libraries/AP_Motors/AP_MotorsCoax.cpp` | GPL-3.0 |
| `libraries__GCS_MAVLink__GCS.cpp` | `libraries/GCS_MAVLink/GCS.cpp` | GPL-3.0 |
| `libraries__GCS_MAVLink__GCS_Common.cpp` | `libraries/GCS_MAVLink/GCS_Common.cpp` | GPL-3.0 |
| `libraries__GCS_MAVLink__GCS_MAVLink.cpp` | `libraries/GCS_MAVLink/GCS_MAVLink.cpp` | GPL-3.0 |
| `libraries__GCS_MAVLink__GCS_MAVLink_Parameters.cpp` | `libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp` | GPL-3.0 |
| `libraries__GCS_MAVLink__GCS_Param.cpp` | `libraries/GCS_MAVLink/GCS_Param.cpp` | GPL-3.0 |
| `libraries__SRV_Channel__SRV_Channel.cpp` | `libraries/SRV_Channel/SRV_Channel.cpp` | GPL-3.0 |
| `libraries__SRV_Channel__SRV_Channel.h` | `libraries/SRV_Channel/SRV_Channel.h` | GPL-3.0 |

- **Источник:** `https://github.com/ArduPilot/ArduPilot`, тег `Copter-4.7.2`,
  коммит `fb773567896014da007bca58358704d4a0d8516f`.
- **Содержимое файлов не изменялось.** Переименование — только в имени файла
  (`путь/к/файлу` → `путь__к__файлу`), чтобы сохранить структуру источника.
  Внутренние заголовки с копирайтом сохранены как есть.
- **Текст лицензии:** `research/ardupilot/COPYING.txt` (GPL-3.0, ArduPilot).
- **Проверка:** `python code/verify_vendored_sources.py` перекачивает эти же
  файлы из upstream по указанному коммиту и сверяет SHA-256. Публикуемые
  файлы совпадают побайтово.

Прочие материалы ArduPilot (документация `ardupilot.org`, схемы рамы) — авторские
права ArduPilot, используются как ссылка в `docs/`.

## Производные рабочие артефакты

| Файл | Что это |
|---|---|
| `research/ardupilot/frames_parsed.json` | Разобранные определения классов и типов рамы, собранные во время исследования. Это **не** побайтовая копия файла upstream, поэтому в проверку verbatim не входит. Первичный источник определений — `libraries/AP_Motors/AP_Motors_Class.h` и схемы рамы в документации ArduPilot. |
| `research/gui_preview.png`, `research/screenshots/*.png` | Скриншоты собственного приложения, сделаны в ходе проверки GUI. |

## Что не публикуется (осталось локально в рабочем каталоге)

Эти материалы использовались в исследовании, но в публичный репозиторий не
входят; список зафиксирован в `.gitignore`.

| Материал | Где взять | Лицензия | Зачем был нужен |
|---|---|---|---|
| `research/px4_src/` | `https://github.com/PX4/PX4-Autopilot` (v1.14.0…v1.16.0) | BSD-3-Clause | Сверка того, как PX4 решает ту же задачу, и различение подходов. Проект переведён на ArduPilot, сверка велась с ним. |
| `research/mavlink/` (`common.xml`, `common_crc.xml`, `pymavlink_ref.py`, `mavlink_parse.py`) | `https://github.com/mavlink/mavlink`, `https://github.com/ArduPilot/pymavlink` | GPL-3.0 | Официальный диалект и эталонная реализация для побайтной кросс-проверки собственного кодека. |
| `research/docs_actuators_v1.15.txt` | `https://docs.px4.io` | BSD-3-Clause | Справка PX4 по актуаторам. |
| `research/ardupilot_tag/` | то же, что `research/ardupilot/` | GPL-3.0 | Дубликат подмножества тех же файлов. |
| `!!!px4.zip` | — | — | Архив прежней сессии PX4; содержимое побайтово продублировано в `code/`, `docs/`, `logs/`. |

### Как вернуть кросс-проверку с pymavlink

Тест `code/test_mavlink_codec.py` ищет эталонный модуль в переменной
`MAVLINK_REF`, затем в `%TEMP%\pymavlink_ref.py` и в `research/mavlink/`.
Если эталон не найден, кросс-проверка **пропускается** (SKIP), а не падает.
Чтобы её включить, положите `pymavlink_ref.py` (сгенерированный `mavgen.py` из
официального `common.xml`) в `research/mavlink/` или укажите путь в `MAVLINK_REF`.
