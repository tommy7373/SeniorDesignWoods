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
//  Adafruit_7segment green_seg = Adafruit_7segment();
Servo myservo;

unsigned long currMs = 0;
unsigned long lastUpdateMs;

/////////////////////
// SD Card Globals //
/////////////////////
bool sdCardPresent = false; // Keeps track of if SD card is plugged in
String logFileName = "mainlog.csv"; // Active logging file
String logFileBuffer; // Buffer for logged data. Max is set in config

const int numReadings = 75; // Number of samples to take for averaging

int readings_X[numReadings]; // Variables for averaging
int readIndex_X = 0;
int total_X = 0;
int average_X = 0;
int readings_Y[numReadings];
int readIndex_Y = 0;
int total_Y = 0;
int average_Y = 0;
int readings_Z[numReadings];
int readIndex_Z = 0;
int total_Z = 0;
int average_Z = 0;

int pos = 0;

//Setup for poll rates
unsigned short accelFSR = 16; // Set accel full-scale range: options are 2, 4, 8, or 16 g 
unsigned short gyroFSR = 2000; // Set gyro full-scale range: options are 250, 500, 1000, or 2000 deg/s
unsigned short UpdateRate = 200; // Set DMP/IMU update rate, between 4-200Hz
unsigned short imuLPF = 5; // Set gyro/accel LPF: options are 5, 10, 20, 42, 98, 188 Hz

void setup() {
  SERIAL_PORT_USBVIRTUAL.begin(115200);
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);
  pinMode(HW_LED_PIN, OUTPUT);
  digitalWrite(HW_LED_PIN, LOW);
  if(initSD())
  {
    sdCardPresent = true;
  }
  myservo.attach(10); // Servo attached on pin 10

  initIMU();
}

void loop() {
  currMs = millis();
  imu.dmpUpdateFifo();
  lastUpdateMs = currMs;
  
  total_X = total_X - readings_X[readIndex_X]; // Calculating average for the X axis
  readings_X[readIndex_X] = imu.ax;
  total_X = total_X + readings_X[readIndex_X];
  readIndex_X = readIndex_X+1;
  if (readIndex_X >= numReadings){
    readIndex_X=0;
  }
  average_X = total_X / numReadings;

  total_Y = total_Y - readings_Y[readIndex_Y]; // Calculating average for the Y axis
  readings_Y[readIndex_Y] = imu.ay;
  total_Y = total_Y + readings_Y[readIndex_Y];
  readIndex_Y = readIndex_Y+1;
  if (readIndex_Y >= numReadings){
    readIndex_Y=0;
  }
  average_Y = total_Y / numReadings;

  total_Z = total_Z - readings_Z[readIndex_Z]; // Calculating average for the Z axis
  readings_Z[readIndex_Z] = imu.az;
  total_Z = total_Z + readings_Z[readIndex_Z];
  readIndex_Z = readIndex_Z+1;
  if (readIndex_Z >= numReadings){
    readIndex_Z=0;
  }
  average_Z = total_Z / numReadings;
  
  pos = abs(average_X/182)+1;
  myservo.write(pos);
  
  logData();
  delay(5);
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
  //imuLog += "Accel (x,y,z): ";
  imuLog += String(average_X) + ", "; // Logging using csv format
  imuLog += String(average_Y) + ", ";
  imuLog += String(average_Z) + ", ";
  imuLog += "\r\n";
//  if(sdCardPresent = true){
//    File logFile = SD.open(logFileName, FILE_WRITE);
//    if(logFile){
//      logFile.print(imuLog);
//      logFile.close();
//    }
//  }
  SERIAL_PORT_USBVIRTUAL.print(imuLog); // Printing to USB port serial
}

bool initSD(void)
{
  // SD.begin should return true if a valid SD card is present
  if ( !SD.begin(38) )
  {
    return false;
  }

  return true;
}
