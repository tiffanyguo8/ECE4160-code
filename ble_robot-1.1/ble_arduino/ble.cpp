#include "ble.h"

void handle_command(RobotCommand robot_cmd, BLECStringCharacteristic rx_characteristic_string, BLECStringCharacteristic tx_characteristic_string, int* ble_flag, int* pid_flag)
{
    EString tx_estring_value;
    
    long interval = 500;
    static long previousMillis = 0;
    unsigned long currentMillis = 0;

    int count = 0;
    static long begin_time = 0;
    
    int temp = 0;
    
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
            int int_a, int_b;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_b);
            if (!success)
                return;

            Serial.print("Two Integers: ");
            Serial.print(int_a);
            Serial.print(", ");
            Serial.println(int_b);
            
            break;
        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:
            /*
             * Your code goes here.
             */

            break;
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO:

            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;

            /*
             * Your code goes here.
             */
            Serial.print("Robot says -> ");
            char buffer[40];
            sprintf(buffer, "%s", char_arr);
            Serial.print(buffer);
            Serial.println(" :)");
            
            tx_estring_value.clear();
            tx_estring_value.append("Robot says -> ");
            tx_estring_value.append(buffer);
            tx_estring_value.append(" :)");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        /*
         * DANCE
         */
        case DANCE:
            Serial.println("Look Ma, I'm Dancin'!");

            break;
        
        /*
         * SET_VEL
         */
        case SET_VEL:

            break;
        
        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        
            
        case GET_TIME_MILLIS:
            Serial.print("Sending: T:");
            Serial.println(millis());
            
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            
            tx_estring_value.append((int)millis());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
            
        case GET_TEMP_5s:            
            Serial.print("Sending timestamped temperature data. ");

            tx_estring_value.clear();
            for (int i = 0; i < 5; i++)
            {
                tx_estring_value.append("T:");
                tx_estring_value.append((int)millis());
                tx_estring_value.append("|C:");
                tx_estring_value.append(getTempDegC());
                if (i < 4)
                    tx_estring_value.append("|");
                delay(1000);
            }
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            break;
        
        case GET_TEMP_5s_RAPID:
            Serial.println("Collecting temperature data... ");

            tx_estring_value.clear();
            begin_time = millis();
            while (millis() < begin_time + 5000)
            {
                tx_estring_value.append("T:");
                tx_estring_value.append((int)millis());
                tx_estring_value.append("|C:");
                tx_estring_value.append(getTempDegC());
                tx_estring_value.append("|");
                count+=1;
                if(count == 5)
                {
                  tx_characteristic_string.writeValue(tx_estring_value.c_str());
                  tx_estring_value.clear();
                  count = 0;
                }
            }
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.println("Data transmission complete.");
            break;

//         case GET_DIST: // Get TOF sensor distance readings for specified time
//             Serial.println("Collecting distance data... ");

//             tx_estring_value.clear();
//             begin_time = millis();

//             temp = robot_cmd.get_next_value(tof_duration);
//             if (!temp)
//             {
//               Serial.println("Error: Duration of distance sensing not specified. ");
//               tx_estring_value.append("Error: Duration of distance sensing not specified. ");
//               tx_characteristic_string.writeValue(tx_estring_value.c_str());
//               tx_estring_value.clear();
//               return;
//             }

//             while (millis() < begin_time + tof_duration)
//             {
//               distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement
//               distanceSensor2.startRanging(); //Write configuration bytes to initiate measurement

//               // if(distanceSensor1.checkForDataReady() && distanceSensor2.checkForDataReady())
//               // {
//               while (!distanceSensor1.checkForDataReady() || !distanceSensor2.checkForDataReady())
//               {
//                 delay(1);
//               }
//                 int distance1 = distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
//                 int distance2 = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor

//                 distanceSensor1.clearInterrupt();
//                 distanceSensor1.stopRanging();
//                 distanceSensor2.clearInterrupt();
//                 distanceSensor2.stopRanging();

//                 tx_estring_value.append("T:");
//                 tx_estring_value.append((int)millis());
//                 tx_estring_value.append("|D1:");
//                 tx_estring_value.append(distance1);
//                 tx_estring_value.append("|D2:");
//                 tx_estring_value.append(distance2);
//                 tx_estring_value.append("|");
//                 tx_characteristic_string.writeValue(tx_estring_value.c_str());
//                 Serial.print("       Sending... Distance 1: ");
//                 Serial.print(distance1);
//                 Serial.print("  Distance 2: ");
//                 Serial.println(distance2);
//                 tx_estring_value.clear();
//               //}         
//             }
//             break;
        
//         case GET_IMU: // Send time stamped IMU roll, pitch, and yaw readings
//             Serial.println("Collecting IMU data... ");
            
//             tx_estring_value.clear();
//             begin_time = millis();

//             temp = robot_cmd.get_next_value(imu_duration);
//             if (!temp)
//             {
//               Serial.println("Error: Duration of distance sensing not specified. ");
//               tx_estring_value.append("Error: Duration of distance sensing not specified. ");
//               tx_characteristic_string.writeValue(tx_estring_value.c_str());
//               tx_estring_value.clear();
//               return;
//             }

//             while (millis() < begin_time + imu_duration)
//             {
//               if (myICM.dataReady())
//               {
//                 timestamp_imu = millis();
//                 myICM.getAGMT();
//                 pitch_a = atan2(myICM.accY(),myICM.accZ())*180/M_PI; 
//                 roll_a  = atan2(myICM.accX(),myICM.accZ())*180/M_PI; 

//                 dt = (micros()-last_time)/1000000.;
//                 last_time = micros();
//                 pitch_g = pitch_g + myICM.gyrX()*dt;
//                 roll_g = roll_g + myICM.gyrY()*dt;
//                 yaw_g = yaw_g + myICM.gyrZ()*dt;

//                 pitch = (pitch+myICM.gyrX()*dt)*0.9 + pitch_a*0.1;
//                 roll = (roll+myICM.gyrY()*dt)*0.9 + roll_a*0.1;
//                 yaw = (yaw+myICM.gyrZ()*dt);

//               /** Serial Plotter  
//                 Serial.print(pitch);
//                 Serial.print("      ");
//                 Serial.print(roll);
//                 Serial.print("      ");
//                 Serial.println(yaw);
//               */
//                 tx_estring_value.append("2");
//                 tx_estring_value.append("T:");
//                 tx_estring_value.append((int)millis());
//                 tx_estring_value.append("|P:");
//                 tx_estring_value.append(pitch);
//                 tx_estring_value.append("|R:");
//                 tx_estring_value.append(roll);
//                 tx_estring_value.append("|Y");
//                 tx_estring_value.append(yaw);
//                 tx_estring_value.append("|");
//                 tx_characteristic_string.writeValue(tx_estring_value.c_str());
//                 Serial.print("Sending... Pitch: ");
//                 Serial.print(pitch);
//                 Serial.print("  Roll: ");
//                 Serial.print(roll);
//                 Serial.print("  Yaw: ");
//                 Serial.println(yaw);
//                 tx_estring_value.clear();
//               }
//             break;

//         case GET_TOF_IMU:
//             Serial.println("Collecting TOF and IMU data... ");
//             tx_estring_value.clear();
//             begin_time = millis();

//             temp = robot_cmd.get_next_value(imu_duration);
//             if (!temp)
//             {
//               Serial.println("Error: Duration of distance sensing not specified. ");
//               tx_estring_value.append("Error: Duration of distance sensing not specified. ");
//               tx_characteristic_string.writeValue(tx_estring_value.c_str());
//               tx_estring_value.clear();
//               return;
//             }

//             while (millis() < begin_time + imu_duration)
//             {
//               distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement
//               distanceSensor2.startRanging(); //Write configuration bytes to initiate measurement

//               if (myICM.dataReady())
//               {
//                 timestamp_imu = millis();
//                 myICM.getAGMT();
//                 pitch_a = atan2(myICM.accY(),myICM.accZ())*180/M_PI; 
//                 roll_a  = atan2(myICM.accX(),myICM.accZ())*180/M_PI; 

//                 dt = (micros()-last_time)/1000000.;
//                 last_time = micros();
//                 pitch_g = pitch_g + myICM.gyrX()*dt;
//                 roll_g = roll_g + myICM.gyrY()*dt;
//                 yaw_g = yaw_g + myICM.gyrZ()*dt;

//                 pitch = (pitch+myICM.gyrX()*dt)*0.9 + pitch_a*0.1;
//                 roll = (roll+myICM.gyrY()*dt)*0.9 + roll_a*0.1;
//                 yaw = (yaw+myICM.gyrZ()*dt);

//               /** Serial Plotter  
//                 Serial.print(pitch);
//                 Serial.print("      ");
//                 Serial.print(roll);
//                 Serial.print("      ");
//                 Serial.println(yaw);
//               */
//                 tx_estring_value.append("2");
//                 tx_estring_value.append("T:");
//                 tx_estring_value.append((int)millis());
//                 tx_estring_value.append("|P:");
//                 tx_estring_value.append(pitch);
//                 tx_estring_value.append("|R:");
//                 tx_estring_value.append(roll);
//                 tx_estring_value.append("|Y:");
//                 tx_estring_value.append(yaw);
//                 tx_estring_value.append("|");
//                 tx_characteristic_string.writeValue(tx_estring_value.c_str());
//                 // Serial.print("Sending... Pitch: ");
//                 // Serial.print(pitch);
//                 // Serial.print("  Roll: ");
//                 // Serial.print(roll);
//                 // Serial.print("  Yaw: ");
//                 // Serial.println(yaw);
//                 tx_estring_value.clear();
//               }

//               if(distanceSensor1.checkForDataReady() && distanceSensor2.checkForDataReady())
//               {
//                 int distance1 = distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
//                 int distance2 = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor

//                 distanceSensor1.clearInterrupt();
//                 distanceSensor1.stopRanging();
//                 distanceSensor2.clearInterrupt();
//                 distanceSensor2.stopRanging();

//                 tx_estring_value.append("1");
//                 tx_estring_value.append("T:");
//                 tx_estring_value.append((int)millis());
//                 tx_estring_value.append("|D1:");
//                 tx_estring_value.append(distance1);
//                 tx_estring_value.append("|D2:");
//                 tx_estring_value.append(distance2);
//                 tx_estring_value.append("|");
//                 tx_characteristic_string.writeValue(tx_estring_value.c_str());
//                 // Serial.print("       Sending... Distance 1: ");
//                 // Serial.print(distance1);
//                 // Serial.print("  Distance 2: ");
//                 // Serial.println(distance2);
//                 tx_estring_value.clear();
//               }
//             }
//             break;

        case SEND_PWM:
            static char cmd_input[5];
            static int pwm = 0;
            static char direction = '\0';
            temp = robot_cmd.get_next_value(cmd_input);
            if (!temp)
            {
              Serial.println("Error: PWM and direction not specified. ");
              tx_estring_value.append("Error: PWM and direction not specified. ");
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
              tx_estring_value.clear();
              return;
            }

            direction = cmd_input[0];
            // Serial.println(direction);
            cmd_input[0] = '0';
            pwm = atoi(cmd_input);
            // Serial.println(pwm);

            switch(direction){
              case 'F':
                    Serial.println("Going forward");
                  forward(pwm);
                  break;
              case 'B':
                    Serial.println("Going backward");
                  backward(pwm);
                  break;
              case 'L':
                    Serial.println("Turning left");
                  left(pwm);
                  break;
              case 'R':
                    Serial.println("Turning right");
                  right(pwm);
                  break;
              case 'S':
                    Serial.println("Stopping");
                  stop();
                  break;
              default:
                  Serial.print("Motor switch case got unexpected letter (not F, B, L, R, S): ");
                  Serial.println(direction);
                  stop();
                  break;
            }

            break;
            
        case PID:
            Serial.println("Performing PID - drive to 1 ft of wall and stop. ");
            temp = robot_cmd.get_next_value(cmd_input);
            Serial.println(temp);
            *pid_flag = 1;
            break;
            
        case SEND_DATA:
            *ble_flag = 1;
            break;

        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
            }
  }