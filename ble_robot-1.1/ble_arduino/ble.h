#include "RobotCommand.h"
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "motor.h"

enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS,
    GET_TEMP_5s,
    GET_TEMP_5s_RAPID,
    GET_DIST,
    GET_IMU,
    GET_TOF_IMU,
    SEND_PWM,
    PID,
    SEND_DATA,
};

// void write_data(BLEFloatCharacteristic tx_characteristic_float);
// void read_data(BLECStringCharacteristic rx_characteristic_string, RobotCommand robot_cmd, BLECStringCharacteristic tx_characteristic_string);
void handle_command(RobotCommand robot_cmd, BLECStringCharacteristic rx_characteristic_string, BLECStringCharacteristic tx_characteristic_string, int* ble_flag, int* pid_flag);