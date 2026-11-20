"""Таблицы рамы ArduCopter 4.7.2 (справочные данные).

Сгенерировано из исходников ArduPilot 4.7.2
(commit fb773567896014da007bca58358704d4a0d8516f):
  libraries/AP_Motors/AP_MotorsMatrix.cpp  -> setup_<class>_matrix()
  libraries/AP_Motors/AP_Motors_Class.h     -> enum motor_frame_class / motor_frame_type

Каждая запись мотора: (угол_градусы, направление, порядок_в_тесте).
Угол отсчитывается от носа по часовой стрелке: 0 = перед, 90 = право,
180 = зад, -90 = лево. Для рамы X: 45 = передний правый, 135 = задний правый,
-135 = задний левый, -45 = передний левый.
Направление: CCW = против часовой стрелки, CW = по часовой стрелке.

Не редактировать вручную.
"""

from __future__ import annotations

# FRAME_CLASS -> имя класса рамы (AP_Motors_Class.h)
FRAME_CLASS = {
    0: 'UNDEFINED',
    1: 'QUAD',
    2: 'HEXA',
    3: 'OCTA',
    4: 'OCTAQUAD',
    5: 'Y6',
    6: 'HELI',
    7: 'TRI',
    8: 'SINGLE',
    9: 'COAX',
    10: 'TAILSITTER',
    11: 'HELI_DUAL',
    12: 'DODECAHEXA',
    13: 'HELI_QUAD',
    14: 'DECA',
    15: 'SCRIPTING_MATRIX',
    16: '6DOF_SCRIPTING',
    17: 'DYNAMIC_SCRIPTING_MATRIX',
}

# FRAME_TYPE -> имя типа рамы (AP_Motors_Class.h)
FRAME_TYPE = {
    0: 'PLUS',
    1: 'X',
    2: 'V',
    3: 'H',
    4: 'VTAIL',
    5: 'ATAIL',
    6: 'PLUSREV',
    10: 'Y6B',
    11: 'Y6F',
    12: 'BF_X',
    13: 'DJI_X',
    14: 'CW_X',
    15: 'I',
    16: 'NYT_PLUS',
    17: 'NYT_X',
    18: 'BF_X_REV',
    19: 'Y4',
    20: 'X_COR',
    21: 'CW_X_COR',
}

# FRAME_CLASS -> {FRAME_TYPE: [(угол, направление, порядок в тесте), ...]}
FRAME_LAYOUTS = {
    1: {  # QUAD
        'name': 'QUAD',
        'types': {
            0: [  # MOTOR_FRAME_TYPE_PLUS
                (90.0, 'CCW', 2),  # M1
                (-90.0, 'CCW', 4),  # M2
                (0.0, 'CW', 1),  # M3
                (180.0, 'CW', 3),  # M4
            ],
            1: [  # MOTOR_FRAME_TYPE_X
                (45.0, 'CCW', 1),  # M1
                (-135.0, 'CCW', 3),  # M2
                (-45.0, 'CW', 4),  # M3
                (135.0, 'CW', 2),  # M4
            ],
            16: [  # MOTOR_FRAME_TYPE_NYT_PLUS
                (90.0, 'FLOAT:0', 2),  # M1
                (-90.0, 'FLOAT:0', 4),  # M2
                (0.0, 'FLOAT:0', 1),  # M3
                (180.0, 'FLOAT:0', 3),  # M4
            ],
            17: [  # MOTOR_FRAME_TYPE_NYT_X
                (45.0, 'FLOAT:0', 1),  # M1
                (-135.0, 'FLOAT:0', 3),  # M2
                (-45.0, 'FLOAT:0', 4),  # M3
                (135.0, 'FLOAT:0', 2),  # M4
            ],
            12: [  # MOTOR_FRAME_TYPE_BF_X
                (135.0, 'CW', 2),  # M1
                (45.0, 'CCW', 1),  # M2
                (-135.0, 'CCW', 3),  # M3
                (-45.0, 'CW', 4),  # M4
            ],
            18: [  # MOTOR_FRAME_TYPE_BF_X_REV
                (135.0, 'CCW', 2),  # M1
                (45.0, 'CW', 1),  # M2
                (-135.0, 'CW', 3),  # M3
                (-45.0, 'CCW', 4),  # M4
            ],
            13: [  # MOTOR_FRAME_TYPE_DJI_X
                (45.0, 'CCW', 1),  # M1
                (-45.0, 'CW', 4),  # M2
                (-135.0, 'CCW', 3),  # M3
                (135.0, 'CW', 2),  # M4
            ],
            14: [  # MOTOR_FRAME_TYPE_CW_X
                (45.0, 'CCW', 1),  # M1
                (135.0, 'CW', 2),  # M2
                (-135.0, 'CCW', 3),  # M3
                (-45.0, 'CW', 4),  # M4
            ],
            2: [  # MOTOR_FRAME_TYPE_V
                (45.0, 'FLOAT:0.7981f', 1),  # M1
                (-135.0, 'FLOAT:1.0000f', 3),  # M2
                (-45.0, 'FLOAT:-0.7981f', 4),  # M3
                (135.0, 'FLOAT:-1.0000f', 2),  # M4
            ],
            3: [  # MOTOR_FRAME_TYPE_H
                (45.0, 'CW', 1),  # M1
                (-135.0, 'CW', 3),  # M2
                (-45.0, 'CCW', 4),  # M3
                (135.0, 'CCW', 2),  # M4
            ],
            6: [  # MOTOR_FRAME_TYPE_PLUSREV
                (90.0, 'CW', 2),  # M1
                (-90.0, 'CW', 4),  # M2
                (0.0, 'CCW', 1),  # M3
                (180.0, 'CCW', 3),  # M4
            ],
        },
    },
    2: {  # HEXA
        'name': 'HEXA',
        'types': {
            0: [  # MOTOR_FRAME_TYPE_PLUS
                (0.0, 'CW', 1),  # M1
                (180.0, 'CCW', 4),  # M2
                (-120.0, 'CW', 5),  # M3
                (60.0, 'CCW', 2),  # M4
                (-60.0, 'CCW', 6),  # M5
                (120.0, 'CW', 3),  # M6
            ],
            1: [  # MOTOR_FRAME_TYPE_X
                (90.0, 'CW', 2),  # M1
                (-90.0, 'CCW', 5),  # M2
                (-30.0, 'CW', 6),  # M3
                (150.0, 'CCW', 3),  # M4
                (30.0, 'CCW', 1),  # M5
                (-150.0, 'CW', 4),  # M6
            ],
            13: [  # MOTOR_FRAME_TYPE_DJI_X
                (30.0, 'CCW', 1),  # M1
                (-30.0, 'CW', 6),  # M2
                (-90.0, 'CCW', 5),  # M3
                (-150.0, 'CW', 4),  # M4
                (150.0, 'CCW', 3),  # M5
                (90.0, 'CW', 2),  # M6
            ],
            14: [  # MOTOR_FRAME_TYPE_CW_X
                (30.0, 'CCW', 1),  # M1
                (90.0, 'CW', 2),  # M2
                (150.0, 'CCW', 3),  # M3
                (-150.0, 'CW', 4),  # M4
                (-90.0, 'CCW', 5),  # M5
                (-30.0, 'CW', 6),  # M6
            ],
        },
    },
    3: {  # OCTA
        'name': 'OCTA',
        'types': {
            0: [  # MOTOR_FRAME_TYPE_PLUS
                (0.0, 'CW', 1),  # M1
                (180.0, 'CW', 5),  # M2
                (45.0, 'CCW', 2),  # M3
                (135.0, 'CCW', 4),  # M4
                (-45.0, 'CCW', 8),  # M5
                (-135.0, 'CCW', 6),  # M6
                (-90.0, 'CW', 7),  # M7
                (90.0, 'CW', 3),  # M8
            ],
            1: [  # MOTOR_FRAME_TYPE_X
                (22.5, 'CW', 1),  # M1
                (-157.5, 'CW', 5),  # M2
                (67.5, 'CCW', 2),  # M3
                (157.5, 'CCW', 4),  # M4
                (-22.5, 'CCW', 8),  # M5
                (-112.5, 'CCW', 6),  # M6
                (-67.5, 'CW', 7),  # M7
                (112.5, 'CW', 3),  # M8
            ],
            13: [  # MOTOR_FRAME_TYPE_DJI_X
                (22.5, 'CCW', 1),  # M1
                (-22.5, 'CW', 8),  # M2
                (-67.5, 'CCW', 7),  # M3
                (-112.5, 'CW', 6),  # M4
                (-157.5, 'CCW', 5),  # M5
                (157.5, 'CW', 4),  # M6
                (112.5, 'CCW', 3),  # M7
                (67.5, 'CW', 2),  # M8
            ],
            14: [  # MOTOR_FRAME_TYPE_CW_X
                (22.5, 'CCW', 1),  # M1
                (67.5, 'CW', 2),  # M2
                (112.5, 'CCW', 3),  # M3
                (157.5, 'CW', 4),  # M4
                (-157.5, 'CCW', 5),  # M5
                (-112.5, 'CW', 6),  # M6
                (-67.5, 'CCW', 7),  # M7
                (-22.5, 'CW', 8),  # M8
            ],
        },
    },
    4: {  # OCTAQUAD
        'name': 'OCTAQUAD',
        'types': {
            0: [  # MOTOR_FRAME_TYPE_PLUS
                (0.0, 'CCW', 1),  # M1
                (-90.0, 'CW', 7),  # M2
                (180.0, 'CCW', 5),  # M3
                (90.0, 'CW', 3),  # M4
                (-90.0, 'CCW', 8),  # M5
                (0.0, 'CW', 2),  # M6
                (90.0, 'CCW', 4),  # M7
                (180.0, 'CW', 6),  # M8
            ],
            1: [  # MOTOR_FRAME_TYPE_X
                (45.0, 'CCW', 1),  # M1
                (-45.0, 'CW', 7),  # M2
                (-135.0, 'CCW', 5),  # M3
                (135.0, 'CW', 3),  # M4
                (-45.0, 'CCW', 8),  # M5
                (45.0, 'CW', 2),  # M6
                (135.0, 'CCW', 4),  # M7
                (-135.0, 'CW', 6),  # M8
            ],
            2: [  # MOTOR_FRAME_TYPE_V
                (45.0, 'FLOAT:0.7981f', 1),  # M1
                (-45.0, 'FLOAT:-0.7981f', 7),  # M2
                (-135.0, 'FLOAT:1.0000f', 5),  # M3
                (135.0, 'FLOAT:-1.0000f', 3),  # M4
                (-45.0, 'FLOAT:0.7981f', 8),  # M5
                (45.0, 'FLOAT:-0.7981f', 2),  # M6
                (135.0, 'FLOAT:1.0000f', 4),  # M7
                (-135.0, 'FLOAT:-1.0000f', 6),  # M8
            ],
            3: [  # MOTOR_FRAME_TYPE_H
                (45.0, 'CW', 1),  # M1
                (-45.0, 'CCW', 7),  # M2
                (-135.0, 'CW', 5),  # M3
                (135.0, 'CCW', 3),  # M4
                (-45.0, 'CW', 8),  # M5
                (45.0, 'CCW', 2),  # M6
                (135.0, 'CW', 4),  # M7
                (-135.0, 'CCW', 6),  # M8
            ],
            14: [  # MOTOR_FRAME_TYPE_CW_X
                (45.0, 'CCW', 1),  # M1
                (45.0, 'CW', 2),  # M2
                (135.0, 'CW', 3),  # M3
                (135.0, 'CCW', 4),  # M4
                (-135.0, 'CCW', 5),  # M5
                (-135.0, 'CW', 6),  # M6
                (-45.0, 'CW', 7),  # M7
                (-45.0, 'CCW', 8),  # M8
            ],
            12: [  # MOTOR_FRAME_TYPE_BF_X
                (135.0, 'CW', 3),  # M1
                (45.0, 'CCW', 1),  # M2
                (-135.0, 'CCW', 5),  # M3
                (-45.0, 'CW', 7),  # M4
                (135.0, 'CCW', 4),  # M5
                (45.0, 'CW', 2),  # M6
                (-135.0, 'CW', 6),  # M7
                (-45.0, 'CCW', 8),  # M8
            ],
            18: [  # MOTOR_FRAME_TYPE_BF_X_REV
                (135.0, 'CCW', 3),  # M1
                (45.0, 'CW', 1),  # M2
                (-135.0, 'CW', 5),  # M3
                (-45.0, 'CCW', 7),  # M4
                (135.0, 'CW', 4),  # M5
                (45.0, 'CCW', 2),  # M6
                (-135.0, 'CCW', 6),  # M7
                (-45.0, 'CW', 8),  # M8
            ],
            20: [  # MOTOR_FRAME_TYPE_X_COR
                (45.0, 'CCW', 1),  # M1
                (-45.0, 'CW', 7),  # M2
                (-135.0, 'CCW', 5),  # M3
                (135.0, 'CW', 3),  # M4
                (-45.0, 'CW', 8),  # M5
                (45.0, 'CCW', 2),  # M6
                (135.0, 'CW', 4),  # M7
                (-135.0, 'CCW', 6),  # M8
            ],
            21: [  # MOTOR_FRAME_TYPE_CW_X_COR
                (45.0, 'CCW', 1),  # M1
                (45.0, 'CCW', 2),  # M2
                (135.0, 'CW', 3),  # M3
                (135.0, 'CW', 4),  # M4
                (-135.0, 'CCW', 5),  # M5
                (-135.0, 'CCW', 6),  # M6
                (-45.0, 'CW', 7),  # M7
                (-45.0, 'CW', 8),  # M8
            ],
        },
    },
    12: {  # DODECAHEXA
        'name': 'DODECAHEXA',
        'types': {
            0: [  # MOTOR_FRAME_TYPE_PLUS
                (0.0, 'CCW', 1),  # M1
                (0.0, 'CW', 2),  # M2
                (60.0, 'CW', 3),  # M3
                (60.0, 'CCW', 4),  # M4
                (120.0, 'CCW', 5),  # M5
                (120.0, 'CW', 6),  # M6
                (180.0, 'CW', 7),  # M7
                (180.0, 'CCW', 8),  # M8
                (-120.0, 'CCW', 9),  # M9
                (-120.0, 'CW', 10),  # M10
                (-60.0, 'CW', 11),  # M11
                (-60.0, 'CCW', 12),  # M12
            ],
            1: [  # MOTOR_FRAME_TYPE_X
                (30.0, 'CCW', 1),  # M1
                (30.0, 'CW', 2),  # M2
                (90.0, 'CW', 3),  # M3
                (90.0, 'CCW', 4),  # M4
                (150.0, 'CCW', 5),  # M5
                (150.0, 'CW', 6),  # M6
                (-150.0, 'CW', 7),  # M7
                (-150.0, 'CCW', 8),  # M8
                (-90.0, 'CCW', 9),  # M9
                (-90.0, 'CW', 10),  # M10
                (-30.0, 'CW', 11),  # M11
                (-30.0, 'CCW', 12),  # M12
            ],
        },
    },
    5: {  # Y6
        'name': 'Y6',
        'types': {
        },
    },
    14: {  # DECA
        'name': 'DECA',
        'types': {
            0: [  # MOTOR_FRAME_TYPE_PLUS
                (0.0, 'CCW', 1),  # M1
                (36.0, 'CW', 2),  # M2
                (72.0, 'CCW', 3),  # M3
                (108.0, 'CW', 4),  # M4
                (144.0, 'CCW', 5),  # M5
                (180.0, 'CW', 6),  # M6
                (-144.0, 'CCW', 7),  # M7
                (-108.0, 'CW', 8),  # M8
                (-72.0, 'CCW', 9),  # M9
                (-36.0, 'CW', 10),  # M10
            ],
            14: [  # MOTOR_FRAME_TYPE_CW_X
                (18.0, 'CCW', 1),  # M1
                (54.0, 'CW', 2),  # M2
                (90.0, 'CCW', 3),  # M3
                (126.0, 'CW', 4),  # M4
                (162.0, 'CCW', 5),  # M5
                (-162.0, 'CW', 6),  # M6
                (-126.0, 'CCW', 7),  # M7
                (-90.0, 'CW', 8),  # M8
                (-54.0, 'CCW', 9),  # M9
                (-18.0, 'CW', 10),  # M10
            ],
        },
    },
}
