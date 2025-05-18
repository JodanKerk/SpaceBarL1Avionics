/**********************************************
Code For Space Bar Avionics ver. MRC
Author(s): Jodan Kerk
***********************************************/

/*Include Libraries*/
#include <SPI.h>
#include <LoRa.h>

#include <Adafruit_BNO08x.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

/*Declare Pinout*/
#define BNO08X_RESET 8
#define BNO08X_CS 7
#define BNO08X_INT 9
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define LoRa_SS 5
#define LoRa_RESET 4
#define LoRa_DIO0 6

/*Declare Important Constants (TO ADJUST)*/


/*Declare Other Global Constants & Variables*/
#define SEALEVELPRESSURE_HPA (1013.25)

//Packet Tracking
int counter = 0;

//BNO08X Readout
float BNO08X_ACCELEROMETER_X;
float BNO08X_ACCELEROMETER_Y;
float BNO08X_ACCELEROMETER_Z;
float BNO08X_GYROSCOPE_X;
float BNO08X_GYROSCOPE_Y;
float BNO08X_GYROSCOPE_Z;
float BNO08X_MAGNETOMETER_X;
float BNO08X_MAGNETOMETER_Y;
float BNO08X_MAGNETOMETER_Z;

Adafruit_BMP3XX bmp;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t BNO08X_VALUE;

/*Declare Functions*/
void SET_BNO08X_REPORTS(void) {
  Serial.println("Setting desired reports...");
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("ERROR: Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("ERROR: Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("ERROR: Could not enable magnetic field calibrated");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("ERROR: Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_GRAVITY)) {
    Serial.println("ERROR: Could not enable gravity vector");
  }
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("ERROR: Could not enable rotation vector");
  }
  if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
    Serial.println("ERROR: Could not enable raw accelerometer");
  }
  if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
    Serial.println("ERROR: Could not enable raw gyroscope");
  }
  if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
    Serial.println("ERROR: Could not enable raw magnetometer");
  }
}

void READ_BNO08X_DATA(){
  switch (BNO08X_VALUE.sensorId) {
    case SH2_LINEAR_ACCELERATION:
      BNO08X_ACCELEROMETER_X = BNO08X_VALUE.un.linearAcceleration.x;
      BNO08X_ACCELEROMETER_Y = BNO08X_VALUE.un.linearAcceleration.y;
      BNO08X_ACCELEROMETER_Z = BNO08X_VALUE.un.linearAcceleration.z;
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      BNO08X_GYROSCOPE_X = BNO08X_VALUE.un.gyroscope.x;
      BNO08X_GYROSCOPE_Y = BNO08X_VALUE.un.gyroscope.y;
      BNO08X_GYROSCOPE_Z = BNO08X_VALUE.un.gyroscope.z;
      break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
      BNO08X_MAGNETOMETER_X = BNO08X_VALUE.un.magneticField.x;
      BNO08X_MAGNETOMETER_Y = BNO08X_VALUE.un.magneticField.y;
      BNO08X_MAGNETOMETER_Z = BNO08X_VALUE.un.magneticField.z;
      break;
  }
}

void PRINT_BNO08X_DATA(){
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    SET_BNO08X_REPORTS();
  }

  if (!bno08x.getSensorEvent(&BNO08X_VALUE)) {
    return;
  }

  Serial.print("Accelerometer - x: ");
  Serial.print(BNO08X_ACCELEROMETER_X);
  Serial.print(" y: ");
  Serial.print(BNO08X_ACCELEROMETER_Y);
  Serial.print(" z: ");
  Serial.println(BNO08X_ACCELEROMETER_Z);

  Serial.print("Gyro - x: ");
  Serial.print(BNO08X_GYROSCOPE_X);
  Serial.print(" y: ");
  Serial.print(BNO08X_GYROSCOPE_Y);
  Serial.print(" z: ");
  Serial.println(BNO08X_GYROSCOPE_Z);

  Serial.print("Magnetic Field - x: ");
  Serial.print(BNO08X_MAGNETOMETER_X);
  Serial.print(" y: ");
  Serial.print(BNO08X_MAGNETOMETER_Y);
  Serial.print(" z: ");
  Serial.println(BNO08X_MAGNETOMETER_Z);

  // Serial.print("Linear Acceration - x: ");
  // Serial.print(BNO08X_VALUE.un.linearAcceleration.x);
  // Serial.print(" y: ");
  // Serial.print(BNO08X_VALUE.un.linearAcceleration.y);
  // Serial.print(" z: ");
  // Serial.println(BNO08X_VALUE.un.linearAcceleration.z);

  // Serial.print("Rotation Vector - r: ");
  // Serial.print(BNO08X_VALUE.un.rotationVector.real);
  // Serial.print(" i: ");
  // Serial.print(BNO08X_VALUE.un.rotationVector.i);
  // Serial.print(" j: ");
  // Serial.print(BNO08X_VALUE.un.rotationVector.j);
  // Serial.print(" k: ");
  // Serial.println(BNO08X_VALUE.un.rotationVector.k);
}

void TRANSMIT_BNO08X_DATA() {

  if (bno08x.wasReset()) {
    LoRa.print("WARNING: BNO08X WAS RESET!");
    SET_BNO08X_REPORTS();
  }

  if (!bno08x.getSensorEvent(&BNO08X_VALUE)) {
    return;
  }

  LoRa.print("Packet:");
  LoRa.println(counter);

  LoRa.print("\"");
  LoRa.print(BNO08X_ACCELEROMETER_X);
  LoRa.print("\",");
  LoRa.print(BNO08X_ACCELEROMETER_Y);
  LoRa.print("\",");
  LoRa.print(BNO08X_ACCELEROMETER_Z);
  LoRa.print("\",");

  LoRa.print("\"");
  LoRa.print(BNO08X_GYROSCOPE_X);
  LoRa.print("\",");
  LoRa.print(BNO08X_GYROSCOPE_Y);
  LoRa.print("\",");
  LoRa.print(BNO08X_GYROSCOPE_Z);
  LoRa.print("\",");

  LoRa.print("\"");
  LoRa.print(BNO08X_MAGNETOMETER_X);
  LoRa.print("\",");
  LoRa.print(BNO08X_MAGNETOMETER_Y);
  LoRa.print("\",");
  LoRa.print(BNO08X_MAGNETOMETER_Z);
  LoRa.print("\",");

  // LoRa.print("\"");
  // LoRa.print(BNO08X_VALUE.un.linearAcceleration.x);
  // LoRa.print("\",");
  // LoRa.print(BNO08X_VALUE.un.linearAcceleration.y);
  // LoRa.print("\",");
  // LoRa.println(BNO08X_VALUE.un.linearAcceleration.z);
  // LoRa.print("\",");

  // LoRa.print("\"");
  // LoRa.print(BNO08X_VALUE.un.rotationVector.real);
  // LoRa.print("\",");
  // LoRa.print(BNO08X_VALUE.un.rotationVector.i);
  // LoRa.print("\",");
  // LoRa.print(BNO08X_VALUE.un.rotationVector.j);
  // LoRa.print("\",");
  // LoRa.println(BNO08X_VALUE.un.rotationVector.k);
  // LoRa.print("\",");
}

void PRINT_BMP388_DATA(){
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature: ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure: ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Altitude: ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
}

void TRANSMIT_BMP388_DATA(){
  if (! bmp.performReading()) {
    LoRa.println("Failed to perform reading :(");
    return;
  }
  LoRa.print("\"");
  LoRa.print(bmp.temperature);
  LoRa.print("\",");

  LoRa.print("\"");
  LoRa.print(bmp.pressure / 100.0);
  LoRa.print("\",");

  LoRa.print("\"");
  LoRa.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  LoRa.print("\",");
}

/*Setup Code, To Run Once*/
void setup() {
  //Initialise and Wait for Serial to Begin
  Serial.begin(9600);
  //while (!Serial); 

  //Initialise External Peripherals
  LoRa.setPins(LoRa_SS, LoRa_RESET, LoRa_DIO0);
  if (!LoRa.begin(922E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(9);
  LoRa.enableCrc();

  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1);
  }

  if (!bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  SET_BNO08X_REPORTS();
  delay(100);
}

void TRANSMIT_ALL_DATA(){
  if (LoRa.beginPacket()==0){}
  else {
    LoRa.beginPacket();
    TRANSMIT_BNO08X_DATA();
    TRANSMIT_BMP388_DATA();
    LoRa.endPacket(true);
    counter++;
  }
}

/*Main Code*/
void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  READ_BNO08X_DATA();
  PRINT_BNO08X_DATA();
  PRINT_BMP388_DATA();

  TRANSMIT_ALL_DATA();
  delay(100);
}
