# Маппинг моторов ArduCopter 4.7.2: стандарты, проверки, конфигуратор

Дата: 2026-11-20. Прошивка подтверждена по локальному артефакту
`C:\Trading\!!!\QUAV5+\CUAVv5\arducopter.elf`: строки `ArduCopter V4.7.2 (fb773567)`,
`CUAVv5`; `git-version.txt`: commit `fb773567896014da007bca58358704d4a0d8516f`
(ArduPilot, Oct 2026), `firmware-version.txt`: `4.7.2-FIRMWARE_VERSION_TYPE_OFFICIAL`.

Все ссылки на код ниже — на этот конкретный commit.

---

## 1. Что с чем мы имеем дело

| Понятие | Значение | Источник |
|---|---|---|
| Плата | CUAV V5+ | строки `CUAVv5` в `arducopter.elf` |
| Прошивка | ArduCopter V4.7.2 | `firmware-version.txt`, `git-version.txt` |
| Автопилот | `MAV_AUTOPILOT_ARDUPILOTMEGA` (3) | `MAVLink` heartbeat |
| Наша рама | 8 моторов, 4 луча в компоновке квадро-X, по 2 мотора на луч (верхний/нижний), пара на луче вращается встречно | описание пользователя |
| Класс/тип рамы в ArduPilot | `FRAME_CLASS=4` (OCTAQUAD), `FRAME_TYPE=1` (X) | см. раздел 3 |

**Важно:** в ArduPilot 4.7.2 класса «OctoCoax» больше нет. Значения `FRAME_CLASS`
(ArduCopter/Parameters.cpp:683):

```
0:Undefined 1:Quad 2:Hexa 3:Octa 4:OctaQuad 5:Y6 6:Heli 7:Tri
8:SingleCopter 9:CoaxCopter 10:BiCopter 11:Heli_Dual 12:DodecaHexa
13:HeliQuad 14:Deca 15:ScriptingMatrix 16:6DOFScripting 17:DynamicScriptingMatrix
```

`CoaxCopter` (9) — это **вертолет** с двумя моторами (libraries/AP_Motors/AP_MotorsCoax.cpp),
а не восьмимоторная рама. Окто в компоновке квадро — это `OCTAQUAD` (4) с типом `X` (1).

`FRAME_TYPE` (ArduCopter/Parameters.cpp:233): `0:Plus, 1:X, 2:V, 3:H, 4:V-Tail, 5:A-Tail,
10:Y6B, 11:Y6F, 12:BetaFlightX, 13:DJIX, 14:ClockwiseX, 15:I, 18:BetaFlightXReversed, 19:Y4`,
плюс `20:X_COR` и `21:CW_X_COR` в enum (AP_Motors_Class.h) — варианты X8 с
сонаправленным вращением, к нашей раме не относятся.

---

## 2. Как в ArduPilot устроен маппинг «логический мотор → выход»

Цепочка: **логический мотор M1..MN** → **функция SRV-канала** → **физический выход MAIN**.

1. Функция мотора: `libraries/SRV_Channel/SRV_Channel.h:82` —
   `k_motor1 = 33`. То есть M1 → 33, M2 → 34, … M8 → 40
   (`SERVO_FUNCTION_MOTOR1 + N - 1`).
2. Назначение функции на выход: параметр `SERVOx_FUNCTION`
   (x = 1..16). Ноль — выход выключен.
3. Физический смысл номера выхода — разъём `MAIN Mx` платы и провод к ESC.
4. Направление вращения мотора задаётся **раскладкой рамы** (раздел 4), а не
   номером выхода. Если физический мотор на луче крутится не так, как ждёт
   рама, варианты: перевернуть пропеллер/обмотку либо инвертировать сигнал
   выхода параметром `SERVOx_REVERSED=1`.

Следствие: **номер выхода не задаёт, где мотор**, назначение функции — задаёт.
Именно поэтому конфигуратор сначала измеряет физику (тест мотора), а потом
считает, какая функция на каком выходе должна стоять.

---

## 3. Раскладка рамы: откуда берутся позиции и направления

`libraries/AP_Motors/AP_MotorsMatrix.cpp`, функция `setup_octaquad_matrix()`
(строка 973), тип `MOTOR_FRAME_TYPE_X` (строка 993):

```cpp
{   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
{  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7 },
{ -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
{  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
{  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8 },
{   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
{  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
{ -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
```

Третье число — порядок в тесте, а номер мотора = индекс массива
(`add_motors()`: `add_motor(i, motor.angle_degrees, motor.yaw_factor, motor.testing_order)`).
`AP_MOTORS_MATRIX_YAW_FACTOR_CCW = 1`, `..._CW = -1` (AP_MotorsMatrix.h:10-11).

Позиция считается ровно как в `add_motor()` (AP_MotorsMatrix.cpp:537):

```cpp
roll  = cos(radians(angle + 90));
pitch = cos(radians(angle));
```

Отрицательный roll → правая сторона, положительный pitch → перед. Отсюда
раскладка нашей рамы (OCTAQUAD / X):

| Мотор | Луч | Ярус | Направление | Номер теста (param1) |
|---|---|---|---|---|
| M1 | передний правый (FR) | верхний | CCW | 1 |
| M2 | передний левый (FL) | верхний | CW | 7 |
| M3 | задний левый (RL) | верхний | CCW | 5 |
| M4 | задний правый (RR) | верхний | CW | 3 |
| M5 | передний левый (FL) | нижний | CCW | 8 |
| M6 | передний правый (FR) | нижний | CW | 2 |
| M7 | задний правый (RR) | нижний | CCW | 4 |
| M8 | задний левый (RL) | нижний | CW | 6 |

На каждом луче ровно один CCW и один CW — пара на луче вращается встречно.
Это и есть требование коаксиальной схемы: иначе момент по yaw не гасится.

Для сравнения: QUAD/X (та же таблица, `setup_quad_matrix`, строка 576):
M1 = FR/CCW, M2 = RL/CCW, M3 = FL/CW, M4 = RR/CW. Обратите внимание на
нестандартную нумерацию — это не ошибка, а фактический код 4.7.2.

**Уровень (верхний/нижний) в раскладке ArduPilot не кодируется.** Миксеру важны
только луч и направление. Поэтому окта-квадро «верхняя плоскость = M1..M4» и
«верхняя плоскость = M5..M8» для ArduPilot равнозначны; различие только в
нумерации для человека. Наша рама (верхний ПР — CCW, нижний ПР — CW)
полностью совместима с обеими трактовками.

---

## 4. Тест мотора: `MAV_CMD_DO_MOTOR_TEST`

Обработчик: `ArduCopter/GCS_MAVLink_Copter.cpp:734`
(`handle_MAV_CMD_DO_MOTOR_TEST`), реализация — `ArduCopter/motor_test.cpp:145`
(`mavlink_motor_test_start`).

Параметры `COMMAND_LONG` (команда 209):

| Параметр | Значение |
|---|---|
| param1 | **номер ТЕСТА**, а не номер мотора (см. ниже) |
| param2 | тип значения: 0 — процент, 1 — PWM, 2 — газ с RC, 3 — калибровка компаса |
| param3 | значение (для типа 0 — процент) |
| param4 | таймаут, секунды |
| param5 | число моторов в последовательности (0 → 1) |
| param6 | порядок обхода (в 4.7.2 **не используется**) |

### Главная тонкость: param1 — это номер теста, а не номер мотора

ArduPilot ищет мотор по порядковому номеру теста, а не по номеру мотора:

```cpp
void AP_MotorsMatrix::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i] && _test_order[i] == motor_seq) {   // AP_MotorsMatrix.cpp:470
            rc_write(i, pwm);
        }
    }
}
```

`_test_order[motor_num] = testing_order` — это третье число в таблице
`MotorDef` (AP_MotorsMatrix.cpp:523). Для OCTAQUAD / X получаем:

| Мотор | Луч | Направление | Номер теста (param1) | Кнопка в схеме |
|---|---|---|---|---|
| M1 | передний правый | CCW | 1 | A |
| M2 | передний левый | CW | 7 | G |
| M3 | задний левый | CCW | 5 | E |
| M4 | задний правый | CW | 3 | C |
| M5 | передний левый | CCW | 8 | H |
| M6 | передний правый | CW | 2 | B |
| M7 | задний правый | CCW | 4 | D |
| M8 | задний левый | CW | 6 | F |

То есть «кнопка B» (тест №2) крутит **M6**, а не M2. Ошибка здесь не
остаётся безобидной: приложение решило бы, что физический мотор — это M2,
и построило бы неверный план назначения.

**Особенность 4.7.2:** `COMMAND_LONG` конвертируется в `COMMAND_INT`
(`GCS_Common.cpp:5372`, `convert_COMMAND_LONG_to_COMMAND_INT`: `param5 → x`,
`param6 → y`, `param7 → z`), а обработчик коптера берёт число моторов из
`packet.x`. Поэтому `param5` в `COMMAND_LONG` попадает в `x` и трактуется как
число моторов в последовательности. Для одиночного теста достаточно `param5 = 0`,
а `param6` (порядок обхода) в 4.7.2 не используется вовсе.

Типы значения (ArduCopter/motor_test.cpp:15):
`0 = процент, 1 = PWM, 2 = газ с RC, 3 = калибровка компаса`.

Остановка теста: повторная отправка команды с `param4 = 0` —
таймаут обнуляется, `motor_test_output()` сразу вызывает `motor_test_stop()`
(ArduCopter/motor_test.cpp:30-48).

### Ярусы рамы

В прошивке ярус (верхний/нижний мотор) **не кодируется** — миксеру важны луч и
направление. Официальная схема ArduPilot для OCTO QUAD X
(`ardupilot.org/copter/docs/connect-escs-and-motors.html`, рисунок
`m_04_01_octo_quad_x.svg`) задаёт: **верхняя плоскость — M1…M4, нижняя — M5…M8**,
буквы A, G, E, C — верхние, H, B, D, F — нижние. Это согласуется с кодом
(таблица `setup_octaquad_matrix`): на каждом луче один M из верхней и один из
нижней плоскости, вращающиеся встречно.

### Предусловия, которые проверяет сам ArduPilot

`mavlink_motor_control_check()` (ArduCopter/motor_test.cpp:100):

* плата инициализирована;
* `motors->motor_test_checks()` — проверки кадри/параметров;
* RC откалиброван (не требуется только при `param2 = 1`, прямой PWM);
* **аппарат на земле** (`ap.land_complete`);
* **safety switch в положении Safe** (не в « disarm»);
* E-Stop не активен.

При нарушении приходит `COMMAND_ACK` с результатом, отличным от
`MAV_RESULT_ACCEPTED`, плюс `STATUSTEXT` с причиной.

---

## 5. Запись параметров

`GCS_MAVLINK::handle_param_set` (libraries/GCS_MAVLink/GCS_Param.cpp:265):

* параметр ищется по имени; **если его нет — `MAV_PARAM_ERROR_DOES_NOT_EXIST`**;
* значение приходит float32, тип определяется **самим параметром**
  (`AP_Param::find()` → `var_type`), поле `param_type` в `PARAM_SET` игнорируется;
* запись идёт **сразу во flash**: `vp->save(force_save)` (GCS_Param.cpp:318) —
  неверный маппинг переживает перезагрузку, откат только из бэкапа параметров;
* подтверждения-ACK для `PARAM_SET` нет: плата отвечает эхом `PARAM_VALUE`
  с текущим значением (плюс `STATUSTEXT` при ошибке).

**Формат значения параметра.** ArduPilot кладёт числовое значение в float32 и
отдельно шлёт `param_type` (`GCS_Param.cpp:332-344`: `packet.param_value = param_value`).
Поэтому целочисленный параметр читается как float с приведением к int.
Запасное соглашение MAVLink (значение в байтах union'а, big-endian, вправо —
как делает pymavlink) поддержано в конфигураторе как fallback.

---

## 6. Версия прошивки: запрос и перезагрузка

### 6.1 Запрос версии (CMD 520)

Параметра `AUTOPILOT_VERSION` в 4.7.2 нет. Версия читается сообщением
`MSG_AUTOPILOT_VERSION` (msgid 148), которое плата шлёт в ответ на команду
`MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` = **520**:

* `param1` **обязан** быть равен 1, иначе `MAV_RESULT_FAILED`
  (`GCS_Common.cpp:5080-5088`: `if (!is_equal(packet.param1,1.0f)) return MAV_RESULT_FAILED;`
  и только затем `send_message(MSG_AUTOPILOT_VERSION)`);
* `COMMAND_LONG` в 4.7.2 конвертируется в `COMMAND_INT`
  (`GCS_Common.cpp:5372-5386`), поэтому `param1` доходит без изменений.

Формат кадра — как в `mavlink_msg_autopilot_version_pack` (pymavlink dialects/v20):

```
<QQIIIIHH8B8B8B18B   (78 байт)
capabilities, uid, flight_sw_version, middleware_sw_version, os_sw_version,
board_version, vendor_id, product_id, flight_custom_version[8],
middleware_custom_version[8], os_custom_version[8], uid2[18]
```

MAVLink 2 обрезает хвост payload из нулей (`uid2` обычно нулевой), поэтому
декодер дополняет недостающие байты нулями.

**Упаковка версии у ArduPilot** (`GCS_Common.cpp:3000-3003`):

```
flight_sw_version = major << 24 | minor << 16 | patch << 8 | fw_type
```

`flight_custom_version` содержит хеш/строку сборки (`version.fw_hash_str`).

### 6.2 Перезагрузка (CMD 246)

`MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` = **246** (`GCS_Common.cpp:3510-3686`):

* `param1` = 1 — перезагрузка, 3 — в загрузчик; другое значение → `UNSUPPORTED`;
* `target_component` должен быть `MAV_COMP_ID_ALL` (0) **или** `mavlink_system.compid`,
  иначе `MAV_RESULT_DENIED`;
* при `soft_armed` перезагрузка отклоняется (`FAILED`), если не подставить
  магическое значение **20190226** в `packet.y` (в `COMMAND_LONG` это `param6`);
* **ACK отправляется ДО перезагрузки** — успешный ACK не значит, что плата
  ещё жива, её надо дождаться заново по heartbeat.

### 6.3 Связка «тест мотора → перезагрузка»

Критично: **сам тест мотора армит аппарат**.

`ArduCopter/motor_test.cpp:165-170`:

```cpp
if (!motors->armed()) {
    motors->armed(true);
    hal.util->set_soft_armed(true);
}
```

`motor_test_stop()` (`motor_test.cpp:201-233`) возвращает `armed(false)` /
`soft_armed(false)`. Значит порядок перед перезагрузкой обязан быть таким:

1. остановить тест (`MAV_CMD_DO_MOTOR_TEST` с `param1 = 0`);
2. дождаться disarm (`hal.util->get_soft_armed()` == false, в heartbeat — снят
   флаг ARMED, `GCS_MAVLink_Copter.cpp:52-53`);
3. только затем `CMD 246`.

Иначе перезагрузка будет отклонена, а форс-перезагрузка (20190226) применять
нельзя: она обходит эту проверку и оставляет моторы в неопределённом состоянии.

**Следствие для интерфейса.** Блокировку «при ARMED» надо проверять **по
состоянию, снятому ДО отправки команды теста**: после старта теста heartbeat
показывает ARMED всегда, потому что его флаг берётся из `motors->armed()`.

---

## 7. Проверки, которые делает конфигуратор

1. **Один мотор — один выход.** Дубли `SERVOx_FUNCTION` → ошибка.
2. **Каждому логическому мотору — свой выход.** Нет выхода под ожидаемый
   (луч, направление) → ошибка.
3. **Пары на луче вращаются встречно.** Оба мотора на луче с одинаковым
   направлением → ошибка (или предупреждение, если разрешена коррекция через
   `SERVOx_REVERSED`).
4. **Неоднозначность.** Если под мотор подходит несколько выходов с одинаковым
   направлением, выбор делается по номеру выхода с пояснением: при одинаковом
   направлении моменты взаимно компенсируются, поэтому результат равнозначен.
5. **Класс/тип рамы на плате** сверяется с выбранным в приложении; расхождение —
   предупреждение, в план попадает запись `FRAME_CLASS`/`FRAME_TYPE`
   (требуется перезагрузка).
6. **Назначения функций 33…40.** Пропущенная функция (мотор не крутится) —
   предупреждение; одна функция на двух выходах — ошибка.
7. **Границы профиля.** Приложение работает с профилем
   `arducopter-4.7-octaquad-x` (OCTA QUAD X, X8). Для других комбинаций
   класса/типа показывается предупреждение, что раскладка X8 не применяется:
   `FRAME_CLASS=3` (OCTA — 8 независимых лучей), `FRAME_TYPE=0/12/14`
   (PLUS / BF_X / CW_X), `FRAME_TYPE=20/21` (co-rotating), `FRAME_CLASS=17`
   (DYNAMIC_SCRIPTING_MATRIX со скриптом Lua).
8. **Версия прошивки.** Запрашивается при подключении (CMD 520); версия вне
   линейки 4.7.x — предупреждение: таблица раскладки проверена для 4.7.2.
9. **Ручной чек-лист вращения** (8 строк) с прогрессом «X/8 проверено»: ACK
   команды не подтверждает вращение, его подтверждает только наблюдение.
10. **Итоговый отчёт** по формату «было → стало → основание → проверка»,
   с readback каждого параметра.

---

## 8. Что подтверждено на устройстве, а что нет

**Подтверждено чтением с платы 2026-10-26** (снимок `configs/px4_params_COM18_20261026-172517.params`):
тогда на плате стоял **PX4** (`SYS_AUTOSTART=12001`, `CA_ROTOR_COUNT=8`,
`PWM_MAIN_FUNC1..8 = 101..108`), а не ArduPilot. Текущая прошивка — ArduCopter 4.7.2
(по файлам в `C:\Trading\!!!\QUAV5+\CUAVv5`).

**Не подтверждено (нужна плата в работе):**

* фактическая раскладка `SERVOx_FUNCTION` на этой плате;
* физическое соответствие моторов лучам (входные данные для конфигуратора);
* фактическое направление вращения каждого мотора;
* приём платой команд `PARAM_SET` и `MAV_CMD_DO_MOTOR_TEST` (проверяется при первом
  подключении: приложение показывает ACK и readback, но записи не производит без
  подтверждения пользователя).

Прошивку по инструкции проекта не прошивали и настройки на живой плате не меняли:
всё, что выше, получено чтением исходников ArduPilot 4.7.2 и офлайн-тестами.

---

## 9. Рекомендуемая целевая конфигурация для нашей рамы

```
FRAME_CLASS = 4      # OctaQuad
FRAME_TYPE  = 1      # X
SERVO1_FUNCTION = 33 ... SERVO8_FUNCTION = 40   # назначения — по результатам идентификации
SERVOx_REVERSED = 0 # если физика собрана правильно
```

Изменение `FRAME_CLASS`/`FRAME_TYPE` требует перезагрузки платы: приложение
делает это отдельным действием («Записать FRAME_CLASS/FRAME_TYPE») — бэкап
параметров → запись → readback → останов теста мотора → ожидание disarm →
`CMD 246` → ожидание heartbeat после перезагрузки → повторное чтение и сверка
`FRAME_CLASS`/`FRAME_TYPE` с записанными значениями.

### 9.1 Границы профиля `arducopter-4.7-octaquad-x`

Профиль покрывает **только** OCTO QUAD X (X8) на ArduCopter 4.7.x:
`FRAME_CLASS=4` + `FRAME_TYPE=1`. Остальные комбинации в прошивке существуют,
но раскладка X8 к ним неприменима:

| Комбинация | Что это в 4.7.2 | Почему не наш профиль |
|---|---|---|
| `FRAME_CLASS=3` (`OCTA`) | 8 независимых лучей, типы PLUS/X/DJI_X/CW_X | нет коаксиальных пар, другая геометрия |
| `FRAME_CLASS=4`, `FRAME_TYPE=0` | OCTO QUAD PLUS | другая раскладка лучей |
| `FRAME_CLASS=4`, `FRAME_TYPE=12` | OCTO QUAD X (BF_X) | раскладка Betaflight |
| `FRAME_CLASS=4`, `FRAME_TYPE=14` | OCTO QUAD X (CW_X) | пары по лучу, а не по ярусу X8 |
| `FRAME_CLASS=4`, `FRAME_TYPE=20/21` | X_COR / CW_X_COR | co-rotating: в официальной документации этот режим включается через `FRAME_CLASS=17` + скрипт `X8-corotating.lua` |
| `FRAME_CLASS=17` | `MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX` | требует Lua-скрипта, в профиль не входит |

В 4.7.2 альтернативный co-rotating-путь есть и в самом коде:
`AP_MotorsMatrix.cpp:1087-1129` — типы `MOTOR_FRAME_TYPE_X_COR` (20) и
`MOTOR_FRAME_TYPE_CW_X_COR` (21) внутри класса OCTAQUAD, с общим
`AP_MOTORS_FRAME_OCTAQUAD_COROTATING_SCALE_FACTOR`, масштабирующим верхний
слой, чтобы не было биений между ярусами. Это **другой** режим, чем
`FRAME_CLASS=17` из документации, и приложение его автоматически не выбирает.

---

## 10. Ссылки

* ArduPilot 4.7.2: `libraries/AP_Motors/AP_MotorsMatrix.cpp`, `AP_Motors_Class.h`,
  `AP_MotorsCoax.cpp`, `libraries/SRV_Channel/SRV_Channel.h`,
  `libraries/GCS_MAVLink/GCS_Param.cpp`, `GCS_Common.cpp`,
  `ArduCopter/GCS_MAVLink_Copter.cpp`, `ArduCopter/motor_test.cpp`,
  `ArduCopter/Parameters.cpp`
* MAVLink: официальный диалект `common.xml` (pymavlink 2.4.50), CRC_EXTRA сверены
  с сгенерированной реализацией — см. `code/test_mavlink_codec.py`
* PX4 (для контраста, плата была на PX4 26.10): `src/modules/control_allocator/module.yaml`,
  `ROMFS/px4fmu_common/init.d/airframes/12001_octo_cox`,
  `src/modules/commander/Commander.cpp` (`handleCommandActuatorTest`)
