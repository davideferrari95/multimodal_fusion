# Fused Commands
EMPTY_COMMAND                       = 0
PLACE_OBJECT_GIVEN_AREA             = 1
PLACE_OBJECT_GIVEN_AREA_POINT_AT    = 2
PLACE_OBJECT_POINT_AT               = 3
POINT_AT                            = 4
STOP                                = 5
PAUSE                               = 6

# Voice-Only Commands
START_EXPERIMENT                    = 7
OBJECT_MOVED                        = 8
USER_MOVED                          = 9
USER_CANT_MOVE                      = 10
REPLAN_TRAJECTORY                   = 11
WAIT_FOR_COMMAND                    = 12
CAN_GO                              = 13
WAIT_TIME                           = 14

# Errors
NO_ERROR                            = 100
OBSTACLE_DETECTED_ERROR             = 101
MOVE_TO_USER_ERROR                  = 102
ROBOT_STOPPED_SCALING_ERROR         = 103

# Handover Errors
OPEN_GRIPPER_ERROR                  = 201
MOVE_OVER_OBJECT_ERROR              = 202
MOVE_TO_OBJECT_ERROR                = 203
CLOSE_GRIPPER_ERROR                 = 204
MOVE_OVER_OBJECT_AFTER_ERROR        = 205
MOVE_OVER_PLACE_ERROR               = 206
MOVE_TO_PLACE_ERROR                 = 207
OPEN_GRIPPER_AFTER_ERROR            = 208
MOVE_OVER_PLACE_AFTER_ERROR         = 209
MOVE_HOME_ERROR                     = 210

fused_command_info = [
    'EMPTY_COMMAND',
    'PLACE_OBJECT_GIVEN_AREA',
    'PLACE_OBJECT_GIVEN_AREA_POINT_AT',
    'PLACE_OBJECT_POINT_AT',
    'POINT_AT',
    'STOP',
    'PAUSE'
]

voice_only_command_info = [
    'START_EXPERIMENT',
    'OBJECT_MOVED',
    'USER_MOVED',
    'USER_CANT_MOVE',
    'REPLAN_TRAJECTORY',
    'WAIT_FOR_COMMAND',
    'CAN_GO',
    'WAIT_TIME'
]

# Gestures
available_gestures = {
    0: 'NO_GESTURE',
    1: 'PAUSE',
    2: 'POINT_AT',
    3: 'STOP'
}

# Voice Commands
available_voice_commands = {
    0:  'NULL',
    1:  'EXPERIMENT_START',
    2:  'MOVED_OBJECT',
    3:  'PUT_OBJECT_IN_AREA',
    4:  'PUT_OBJECT_IN_GIVEN_AREA',
    5:  'PUT_OBJECT_IN_AREA_GESTURE',
    6:  'USER_MOVED',
    7:  'USER_CANT_MOVE',
    8:  'REPLAN_TRAJECTORY',
    9:  'WAIT_FOR_COMMAND',
    10: 'CAN_GO',
    11: 'WAIT_TIME'
}

# Fused Commands
available_fused_commands = {
    0: 'EMPTY_COMMAND',
    1: 'PLACE_OBJECT_GIVEN_AREA',
    2: 'PLACE_OBJECT_GIVEN_AREA_POINT_AT',
    3: 'PLACE_OBJECT_POINT_AT',
    4: 'POINT_AT',
    5: 'STOP',
    6: 'PAUSE'
}

# Voice-Only Commands
available_voice_only_commands = {
    7:   'START_EXPERIMENT',
    8:   'OBJECT_MOVED',
    9:   'USER_MOVED',
    10:  'USER_CANT_MOVE',
    11:  'REPLAN_TRAJECTORY',
    12:  'WAIT_FOR_COMMAND',
    13:  'CAN_GO',
    14:  'WAIT_TIME'
}
