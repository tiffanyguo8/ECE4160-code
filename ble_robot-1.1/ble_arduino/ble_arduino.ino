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
#define IMU_ARR_LEN 5000

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
  
  distanceSensor1.setDistanceModeShort();
  distanceSensor2.setDistanceModeShort();
    
    distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement
      distanceSensor2.startRanging(); //Write configuration bytes to initiate measurement
    
    // distanceSensor1.setProxIntegrationTime(4);
    // distanceSensor2.setProxIntegrationTime(4);

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

void read_data(int* flag)
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command(robot_cmd, rx_characteristic_string, tx_characteristic_string, flag);
    }
}

int tof_data1[TOF_ARR_LEN];
int tof_data2[TOF_ARR_LEN];
int tof_times1[TOF_ARR_LEN];
int tof_times2[TOF_ARR_LEN];
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
bool imu_overflow = false;
bool speed_overflow = false;

unsigned long last_time = millis();

int speed = 150;
int pid_error = 0;
int setpoint = 400; //304; //304 mm
int curr_dist = 0;
int prev_error = 0;
int accumulator = 0;

void loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());
        
        last_time = millis(); // for IMU

// ------------------------------------------------- While loop ------------------------------------------------- 
        // While central is connected
        while (central.connected()) {
            // Send data
            write_data();

            // Read data
            read_data(&ble_flag);
            
          // distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement
          // distanceSensor2.startRanging(); //Write configuration bytes to initiate measurement
        // Serial.println(tof_counter1);

            if(distanceSensor1.checkForDataReady())
            {
                if(tof_counter1 < TOF_ARR_LEN)
                {
                    int distance1 = distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
                    tof_data1[tof_counter1] = distance1;
                    curr_dist = distance1;
                    tof_times1[tof_counter1] = millis();
                    speed_data[tof_counter1] = speed;
                    distanceSensor1.clearInterrupt();
                    //distanceSensor1.stopRanging();
                    tof_counter1 ++;
                }
                else tof1_overflow = true;
            }
            if(distanceSensor2.checkForDataReady())
            {
                if(tof_counter1 < TOF_ARR_LEN)
                {
                    int distance2 = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor
                    tof_data2[tof_counter2] = distance2;
                    tof_times2[tof_counter2] = millis();
                    distanceSensor2.clearInterrupt();
                    //distanceSensor2.stopRanging();
                    tof_counter2 ++;
                }
                else tof2_overflow = true;
            }

          if (myICM.dataReady())
          {
              if(imu_counter < IMU_ARR_LEN)
              {
                myICM.getAGMT();
                imu_times[imu_counter] = millis();

                pitch_a = atan2(myICM.accY(),myICM.accZ())*180/M_PI; 
                roll_a  = atan2(myICM.accX(),myICM.accZ())*180/M_PI; 

                dt = (micros()-last_time)/1000000.;
                last_time = micros();
                pitch_g = pitch_g + myICM.gyrX()*dt;
                roll_g = roll_g + myICM.gyrY()*dt;
                yaw_g = yaw_g + myICM.gyrZ()*dt;

                pitch = (pitch+myICM.gyrX()*dt)*0.9 + pitch_a*0.1;
                roll = (roll+myICM.gyrY()*dt)*0.9 + roll_a*0.1;
                yaw = (yaw+myICM.gyrZ()*dt);
                imu_pitch[imu_counter] = pitch;
                imu_roll[imu_counter] = roll;
                imu_yaw[imu_counter] = yaw;
                imu_counter ++;
              }
              else imu_overflow = true;
          }
            
        // PID
        if(tof_counter1 > 0) // Make sure we have at least 1 sensor reading
        {
            // Error: Distance from wall - 1ft/304mm
            
            pid_error = curr_dist - setpoint;
            speed = pid(speed, pid_error, prev_error, accumulator);
            prev_error = pid_error;
            accumulator += pid_error;
            
            if(speed == 0) stop();
            else if(speed > 0) forward(speed);
            else backward(-1*speed);
            // Serial.print(speed);
            // Serial.print("\t");
            // Serial.println(curr_dist);
            // Serial.print("\t");
            // Serial.println(pid_error);
        }
            
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
                tx_estring_value.append(speed_data[i]);
                tx_estring_value.append(")");
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
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
            
        // Serial.println(imu_counter);
        // for(int i = 0; i < imu_counter; i++)
        // {
        //     Serial.print(imu_times[i]);
        //     Serial.print('\t');
        //     Serial.print(imu_pitch[i]);
        //     Serial.print('\t');
        //     Serial.print(imu_roll[i]);
        //     Serial.print('\t');
        //     Serial.println(imu_yaw[i]);
        // }
        
        imu_counter = 0;
    }
}