#include <SparkFunMPU9250-DMP.h>
// SD Library manages file and hardware control
#include <SD.h>
// config.h manages default logging parameters and can be used
// to adjust specific parameters of the IMU
#include "config.h"

#include <Servo.h>

#include <SPI.h>
#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

// Flash storage (for nv storage on ATSAMD21)
#ifdef ENABLE_NVRAM_STORAGE
#include <FlashStorage.h>
#endif

MPU9250_DMP imu;
//Adafruit_7segment green_seg = Adafruit_7segment();
Servo myservo;

unsigned long currMs = 0;
unsigned long lastUpdateMs;
int gyro_x, gyro_y, gyro_z;
int accel_x, accel_y, accel_z;

int pos = 0;

//Setup for poll rates
unsigned short accelFSR = 16; // Set accel full-scale range: options are 2, 4, 8, or 16 g 
unsigned short gyroFSR = 250; // Set gyro full-scale range: options are 250, 500, 1000, or 2000 deg/s
unsigned short UpdateRate = 100; // Set DMP/IMU update rate, between 4-200Hz
unsigned short imuLPF = 5; // Set gyro/accel LPF: options are 5, 10, 20, 42, 98, 188 Hz

void setup() {
  SERIAL_PORT_USBVIRTUAL.begin(115200);
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);
  pinMode(HW_LED_PIN, OUTPUT);
  digitalWrite(HW_LED_PIN, LOW);
  myservo.attach(10);

  initIMU();
}

void loop() {
  currMs = millis();
  imu.dmpUpdateFifo();
  lastUpdateMs = currMs;
  accel_x = imu.ax;
  accel_y = imu.ay;
  accel_z = imu.az;
  gyro_x = imu.gx;
  gyro_y = imu.gy;
  gyro_z = imu.gz;
  pos = abs(gyro_z/182)+1;
  myservo.write(pos);
  logData();
  delay(5);
  
//  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(5);                       // waits 15ms for the servo to reach the position
//  }
//  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(5);                       // waits 15ms for the servo to reach the position
//  }
  
//  if (currMs > lastUpdateMs) {
//    imu.dmpUpdateFifo();
//    lastUpdateMs = currMs;
//    accel_x = imu.ax;
//    accel_y = imu.ay;
//    accel_z = imu.az;
//    gyro_x = imu.gx;
//    gyro_y = imu.gy;
//    gyro_z = imu.gz;
//    green_seg.print(imu.calcAccel(accel_y),DEC);
//    
//    green_seg.writeDisplay();
//  }

}

bool initIMU(void)
{
  // imu.begin() should return 0 on success. Will initialize
  // I2C bus, and reset MPU-9250 to defaults.
  if (imu.begin() != INV_SUCCESS)
    return false;

  // Set up MPU-9250 interrupt:
  imu.enableInterrupt(); // Enable interrupt output
  imu.setIntLevel(1);    // Set interrupt to active-low
  imu.setIntLatched(1);  // Latch interrupt output

  // Configure sensors:
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL); // Enable all sensors
  // Set gyro full-scale range: options are 250, 500, 1000, or 2000:
  imu.setGyroFSR(gyroFSR);
  // Set accel full-scale range: options are 2, 4, 8, or 16 g 
  imu.setAccelFSR(accelFSR);
  // Set gyro/accel LPF: options are5, 10, 20, 42, 98, 188 Hz
  imu.setLPF(imuLPF); 

  imu.setSampleRate(UpdateRate);

  // Configure digital motion processor. Use the FIFO to get
  // data from the DMP.
  unsigned short dmpFeatureMask = 0;
  dmpFeatureMask |= DMP_FEATURE_SEND_CAL_GYRO;
  dmpFeatureMask |= DMP_FEATURE_SEND_RAW_ACCEL;
  dmpFeatureMask |= DMP_FEATURE_GYRO_CAL;
  
  // Initialize the DMP, and set the FIFO's update rate:
  imu.dmpBegin(dmpFeatureMask, UpdateRate);

  return true; // Return success
}

void logData(void)
{
  String imuLog = "";
  imuLog += "Gyro (x,y,z): ";
  imuLog += String(imu.gx) + ", ";
  imuLog += String(imu.gy) + ", ";
  imuLog += String(imu.gz) + ", ";
  imuLog += "\r\n";
  SERIAL_PORT_USBVIRTUAL.print(imuLog);
}
