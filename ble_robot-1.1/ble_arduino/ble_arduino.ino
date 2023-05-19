// ----------------------------------------------------------- Bluetooth -----------------------------------------------------------
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
// ---------------------------------------------------------- TOF Sensor -----------------------------------------------------------
#include <Wire.h> // For TOF sensors
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

// ---------------------------------------------------------- IMU Sensor -----------------------------------------------------------
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include<math.h>

// --------------------------------------------------------- Motor/Driver -----------------------------------------------------------
#include "motor.h"
#include "ble.h"
#include "kalman.h"
// ----------------------------------------------------------- Bluetooth -----------------------------------------------------------
//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "014f74b1-658c-430c-be32-b3c35d648b16"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

int count = 0;
// static long begin_time = 0;
//////////// Global Variables ////////////

// ---------------------------------------------------------- TOF Sensor -----------------------------------------------------------
//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 3
#define TOF_ARR_LEN 1000
#define IMU_ARR_LEN 10000

// Shutdown and interrupt pins.
SFEVL53L1X distanceSensor1(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SFEVL53L1X distanceSensor2;

static int tof_duration = 0;
int temp = 0;

// ---------------------------------------------------------- IMU Sensor -----------------------------------------------------------
#define SERIAL_PORT Serial
#define AD0_VAL   1     // The value of the last bit of the I2C address, on the SparkFun 9DoF IMU breakout the default is 1
#define blinkPin LED_BUILTIN

ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
/* Computation variables */
float pitch_a = 0, roll_a = 0, pitch_g = 0, roll_g = 0, yaw_g = 0, dt =0, pitch = 0, roll = 0, yaw = 0;
float Xm = 0, Ym =0, Zm = 0, x = 0, y = 0;
double pitch_a_LPF[] = {0, 0};
const int n =1;
static int imu_duration = 0;
int ble_flag = 0;
int pid_flag = 0;

void setup()
{
    Serial.begin(115200);

    BLE.begin();

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();

// ---------------------------------------------------------- TOF Sensor -----------------------------------------------------------
  Wire.begin();

  Serial.println("VL53L1X Qwiic Test");

  distanceSensor1.sensorOff();
  distanceSensor2.setI2CAddress(0x32);
  distanceSensor1.sensorOn();

    if (distanceSensor1.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }

  if (distanceSensor2.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  
//   distanceSensor1.setDistanceModeShort();
//   distanceSensor2.setDistanceModeShort();
    
    distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement
      distanceSensor2.startRanging(); //Write configuration bytes to initiate measurement

// ---------------------------------------------------------- IMU Sensor -----------------------------------------------------------
  Wire.setClock(400000);
  bool initialized = false;
  while( !initialized )
  {
    myICM.begin( Wire, AD0_VAL );
    Serial.print( F("Initialization of the sensor returned: ") );
    Serial.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok ){
      Serial.println( "Trying again..." );
      delay(500);
    }else{
      initialized = true;
      pinMode(LED_BUILTIN, OUTPUT);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  Serial.println("Sensor online!");

// -------------------------------------------------------- Motors/Drivers ----------------------------------------------------------
    motor_setup();
}

void write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }
        previousMillis = currentMillis;
    }
}

void read_data(int* ble_flag, int* pid_flag)
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command(robot_cmd, rx_characteristic_string, tx_characteristic_string, ble_flag, pid_flag);
    }
}

int tof_data1[TOF_ARR_LEN];
int tof_data2[TOF_ARR_LEN];
int tof_times1[TOF_ARR_LEN];
int tof_times2[TOF_ARR_LEN];
int imu_data[TOF_ARR_LEN];
int kf_times[TOF_ARR_LEN*2];
int kf_counter = 0;
int tof_counter1 = 0;
int tof_counter2 = 0;

int imu_times[IMU_ARR_LEN];
int imu_pitch[IMU_ARR_LEN];
int imu_roll[IMU_ARR_LEN];
int imu_yaw[IMU_ARR_LEN];
int imu_counter = 0;

int speed_data[TOF_ARR_LEN];
int speed_times[TOF_ARR_LEN];
int speed_counter = 0;

bool tof1_overflow = false;
bool tof2_overflow = false;
bool kf_overflow = false;
bool imu_overflow = false;
bool speed_overflow = false;

unsigned long last_time = millis();
unsigned long begin_time = millis();

int speed = 150;
float pid_error = 0;
float pid_accum = 0;
float pid_prev_error = 0;
int setpoint = 400; //304 mm but i changed it to 400 idk
int curr_dist = 0;
int curr_gyr = 0;
int prev_error = 0;
int accumulator = 0;
Matrix<2,1> kalman_dist = 0;
int prev_yaw = 0;
int data_counter = 0;
// int yaw_flag = 0;

int state = 0;
int state_change_flag = 0;
int begin_angle = 0;

int stopped = 0;

// Turn an angle
void angle_turn(int begin_angle, int angle)
{
        if(yaw < begin_angle + angle)
        {
            left(90);
        }
        else
        {
            stop();
            state_change_flag = 1;
        }
}

void loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());
        
        last_time = millis(); // for IMU
        begin_time = millis();

// ------------------------------------------------- While loop ------------------------------------------------- 
        // While central is connected
        while (central.connected()) {
            // Send data
            write_data();

            // Read data
            read_data(&ble_flag, &pid_flag);
            
            if(distanceSensor1.checkForDataReady())
            {
                if(tof_counter1 < TOF_ARR_LEN)
                {
                    int distance1 = distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
                    curr_dist = distance1;
                    
                    tof_data1[tof_counter1] = distance1;
                    tof_times1[tof_counter1] = millis();
                    imu_data[tof_counter1] = yaw;
                    tof_counter1++;
                    
                    // if(yaw >= prev_yaw + 20 && data_counter < 18 && yaw_flag)
                    // {
                    //     tof_data1[tof_counter1] = distance1;
                    //     tof_times1[tof_counter1] = millis();
                    //     imu_data[tof_counter1] = yaw;
                    //     data_counter++;
                    //     prev_yaw = yaw;
                    //     tof_counter1 ++;
                    // }
                    
                    // speed_data[tof_counter1] = speed;
                    distanceSensor1.clearInterrupt();
                }
                else tof1_overflow = true;
                
                // Extrapolation
                // if(tof_counter1 > 0)
                // {
                //     int pred_velocity = (tof_data1[tof_counter1-1] - tof_data1[tof_counter1-2])/(tof_times1[tof_counter1-1] - tof_times1[tof_counter1-2]);
                //     int pred_dist = tof_data1[tof_counter1-1] + pred_velocity * (int(millis())-tof_times1[tof_counter1-1]);
                //     tof_counter1++;
                //     tof_data1[tof_counter1] = pred_dist;
                //     curr_dist = pred_dist;
                //     // Serial.println(pred_dist);
                //     tof_times1[tof_counter1] = millis();
                // }
            }
            if(distanceSensor2.checkForDataReady())
            {
                if(tof_counter2 < TOF_ARR_LEN)
                {
                    int distance2 = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor
                    tof_data2[tof_counter2] = distance2;
                    tof_times2[tof_counter2] = millis();
                    distanceSensor2.clearInterrupt();
                    tof_counter2 ++;
                }
                else tof2_overflow = true;
            }

          if (myICM.dataReady())
          {
              if(imu_counter < IMU_ARR_LEN)
              {
                myICM.getAGMT();

                pitch_a = atan2(myICM.accY(),myICM.accZ())*180/M_PI; 
                roll_a  = atan2(myICM.accX(),myICM.accZ())*180/M_PI; 

                dt = (micros()-last_time)/1000000.;
                last_time = micros();
                pitch_g = pitch_g + myICM.gyrX()*dt;
                roll_g = roll_g + myICM.gyrY()*dt;
                yaw_g = yaw_g + myICM.gyrZ()*dt;
                curr_gyr = myICM.gyrZ();
                // Serial.print("\tIMU time: ");

                pitch = (pitch+myICM.gyrX()*dt)*0.9 + pitch_a*0.1;
                roll = (roll+myICM.gyrY()*dt)*0.9 + roll_a*0.1;
                yaw = (yaw+myICM.gyrZ()*dt);
                // if(!yaw_flag)
                // {
                //     prev_yaw = yaw;
                //     yaw_flag = 1;
                // }
                Serial.println(yaw);
                imu_pitch[imu_counter] = pitch;
                imu_roll[imu_counter] = roll;
                imu_yaw[imu_counter] = yaw;
                imu_times[imu_counter] = millis();
                imu_counter ++;
              }
              else imu_overflow = true;
          }
            
        // For debugging hard fault (isdk why it happens)
        // Serial.print("TOF1: ");
        // Serial.print(tof_counter1);
        // Serial.print("\tTOF2: ");
        // Serial.println(tof_counter2);
            
     /**
        // PID going forward
        if(pid_flag)
        {
            if(tof_counter1 > 0) // Make sure we have at least 1 sensor reading
            {
                // Error: Distance from wall - 1ft/304mm
                kalman_dist = kf(curr_dist, speed);
                if(kf_counter < TOF_ARR_LEN*2)
                {
                    kf_data[kf_counter] = kalman_dist(0,0);
                    kf_times[kf_counter] = millis();
                    kf_counter++;
                }
                else kf_overflow = true;
                //pid_error = curr_dist - setpoint;
                pid_error = -1*kalman_dist(0,0) - setpoint;
                speed = pid(speed, pid_error, prev_error, accumulator);
                prev_error = pid_error;
                accumulator += pid_error;

                if(speed == 0) stop();
                else if(speed > 0) forward(speed);
                else backward(-1*speed);
                // Serial.print(speed);
                // Serial.print("\t");
                // Serial.println(kalman_dist(0,0));
                // Serial.print("\t");
                // Serial.println(pid_error);
            }
        }
      */
    /**    
        // Flip
        if(pid_flag)
        {
            if(stopped)
                stop();
            else if(tof_counter1 > 0) // Make sure we have at least 1 sensor reading
            {
                kalman_dist = kf(curr_dist, speed);
                if(kf_counter < TOF_ARR_LEN*2)
                {
                    kf_data[kf_counter] = kalman_dist(0,0);
                    kf_times[kf_counter] = millis();
                    kf_counter++;
                }
                else kf_overflow = true;
                
                Serial.println(kalman_dist(0,0));
                
                if(abs(kalman_dist(0,0)) > 1500)
                // if(abs(curr_dist) > 1500)
                {
                    forward(255);
                }
                else
                {
                    backward(255);
                    delay(2000);
                    stop();
                    stopped = 1;
                }
                
                // Serial.print(speed);
                // Serial.print("\t");
                // Serial.println(kalman_dist(0,0));
                // Serial.print("\t");
                // Serial.println(pid_error);
            }
        }
    */
    /**   
        // Mapping
        if(pid_flag)
        {
            if(imu_counter > 0) // Make sure we have at least 1 sensor reading
            {
                int gyr_setpoint = 15; // setpoint = 15 degrees/s
                float Kp = -3.5;
                float Ki = -0.25;
                pid_error = curr_gyr - gyr_setpoint;
                int d_e = pid_error - pid_prev_error;
                pid_accum += pid_error*0.01;
                speed = Kp*pid_error + Ki*pid_accum;
                pid_prev_error = pid_error;
                
                // Set upper and lower bounds for speed
                if(speed > 255) speed = 255;
                else if(speed > 0 && speed < 40) speed = 40;
                else if(speed < 0) speed = 0;
                
                left(speed);
                if(yaw > (imu_yaw[0] + 360)) 
                {
                    pid_flag = 0;
                    stop();
                }
            }
        }
    */
            switch(state)
            {
                case 0:
                    forward(80);
                    delay(1000);
                    stop();
                    delay(1000);
                    state_change_flag = 1;
                    break;
                case 1:
                    angle_turn(begin_angle, 308);
                    break;
                case 2:
                    forward(75);
                    delay(1450);
                    stop();
                    delay(1000);
                    state_change_flag = 1;
                    break;
                case 3:
                    angle_turn(begin_angle, 285);
                    break;
                case 4:
                    forward(80);
                    delay(1000);
                    stop();
                    delay(1000);
                    state_change_flag = 1;
                    break;
                case 5:
                    angle_turn(begin_angle, 65);
                    break;
                case 6:
                    forward(80);
                    delay(1400);
                    stop();
                    delay(1000);
                    state_change_flag = 1;
                    break;
                case 7:
                    angle_turn(begin_angle, 87);
                    break;
                case 8:
                    forward(80);
                    delay(750);
                    stop();
                    delay(1000);
                    state_change_flag = 1;
                    break;
                case 9:
                    forward(80);
                    delay(1800);
                    stop();
                    delay(1000);
                    state_change_flag = 1;
                    break;
                case 10:
                    angle_turn(begin_angle, 95);
                    break;
                case 11:
                    forward(80);
                    delay(1600);
                    stop();
                    delay(1000);
                    state_change_flag = 1;
                    break;
                case 12:
                    angle_turn(begin_angle, 95);
                    break;
                case 13:
                    forward(80);
                    delay(1000);
                    stop();
                    delay(1000);
                    state_change_flag = 1;
                    break;
                default:
                    Serial.println("Done or Not a valid state.");
                    stop();
                    break;
            }
            if(state_change_flag)
            {
                begin_angle = yaw;
                state++;
                state_change_flag = 0;
            }
            
        // Sending data to laptop
        if(ble_flag)
        {
            for(int i = 0; i < tof_counter1; i++)
            {
                tx_estring_value.clear();
                tx_estring_value.append("(");
                tx_estring_value.append(tof_times1[i]);
                tx_estring_value.append(",");
                tx_estring_value.append(tof_data1[i]);
                tx_estring_value.append(",");
                tx_estring_value.append(imu_data[i]);
                // if(kf_counter >= i)
                // {
                //     tx_estring_value.append(",");
                //     tx_estring_value.append(kf_data[i]);
                // }
                tx_estring_value.append(")");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            // tx_estring_value.clear();
            // tx_estring_value.append("|Now writing Gyroscope Yaw data|");
            // tx_characteristic_string.writeValue(tx_estring_value.c_str());
            // for(int i = 0; i < imu_counter; i++)
            // {
            //     tx_estring_value.clear();
            //     tx_estring_value.append("(");
            //     tx_estring_value.append(imu_times[i]);
            //     tx_estring_value.append(",");
            //     tx_estring_value.append(imu_yaw[i]);
            //     tx_estring_value.append(")");
            //     tx_characteristic_string.writeValue(tx_estring_value.c_str());
            // }
            ble_flag = 0;
        }
            
        }
// ------------------------------------------------- End loop ------------------------------------------------- 
        Serial.println("Disconnected");
        
        if(tof1_overflow) Serial.println("TOF 1 array overflowed. ");
        if(tof2_overflow) Serial.println("TOF 2 array overflowed. ");
        if(imu_overflow) Serial.println("IMU array overflowed. ");
        
        // Serial.println(tof_counter1);
        // Serial.println(tof_counter2);
        // Serial.println("Distance Sensor 1: ");
        // for(int i = 0; i < tof_counter1; i++)
        // {
        //     Serial.print(tof_times1[i]);
        //     Serial.print('\t');
        //     Serial.println(tof_data1[i]);
        // }
        // Serial.println("Distance Sensor 2: ");
        // for(int i = 0; i < tof_counter2; i++)
        // {
        //     Serial.print(tof_times2[i]);
        //     Serial.print('\t');
        //     Serial.println(tof_data2[i]);
        // }
            
        stop();
        
        tof_counter1 = 0;
        tof_counter2 = 0;
            
        Serial.println(imu_counter);
        for(int i = 0; i < imu_counter; i++)
        {
            Serial.print(imu_times[i]);
            Serial.print('\t');
            Serial.print(imu_pitch[i]);
            Serial.print('\t');
            Serial.print(imu_roll[i]);
            Serial.print('\t');
            Serial.println(imu_yaw[i]);
        }
        
        imu_counter = 0;
    }
}