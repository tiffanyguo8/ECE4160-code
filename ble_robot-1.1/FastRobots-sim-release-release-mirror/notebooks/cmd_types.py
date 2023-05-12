from enum import Enum

class CMD(Enum):
    PING = 0
    SEND_TWO_INTS = 1
    SEND_THREE_FLOATS = 2
    ECHO = 3
    DANCE = 4
    SET_VEL = 5
    GET_TIME_MILLIS = 6
    GET_TEMP_5s = 7
    GET_TEMP_5s_RAPID = 8
    GET_DIST = 9
    GET_IMU = 10
    GET_TOF_IMU = 11
    SEND_PWM = 12
    PID = 13
    SEND_DATA = 14