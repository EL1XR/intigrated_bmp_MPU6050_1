
//format in camel
//put expalaination of functions
//redablity
//modularity(stage wise)
//psudo code
//algo flowchart
//numbering in equation
//

#include <Wire.h>
#include "SparkFunMPL3115A2.h"
#include <SD.h>
#include <SPI.h>
File myFile;
int pinCS = 4;
//Create an instance of the object
MPL3115A2 myPressure;
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw,rollz,pitchz,yawz,Ejt;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime,one,two;
int c = 0;
int a=0;
const int v=7;
void setup() {
  
  Serial.begin(19200);
  pinMode(v,OUTPUT);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
 

  myPressure.begin(); // Get sensor online

  //Configure the sensor
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
  //myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa

  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 

 
  pinMode(pinCS, OUTPUT);
  
  // SD Card Initialization
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }

  delay(2000);
  
}
void loop() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
 accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  rollz=atan(AccY/sqrt((AccX*AccX) + (AccZ*AccZ)));
  //yawz=atan(AccZ/sqrt((AccX*AccX) + (AccY*AccY)));
  pitchz =atan(AccX/sqrt((AccY*AccY) + (AccZ*AccZ)));
  Serial.print("rollz");Serial.print(rollz);
  Serial.print("||||");
  Serial.print("pitchz");Serial.println(pitchz);
 // Serial.print("|||");
  //Serial.print("yawz");Serial.println(yawz);
  Ejt=sqrt((rollz*rollz) + (pitchz*pitchz));
  Serial.print("Ejt:");Serial.println(Ejt);
   a = AccX + AccY + AccZ;
  /*if(Ejt>=0.80 || AccZ<0){
    Serial.println("Parachute");
    digitalWrite(v,HIGH);
    Serial.println("ukkeyyyy");
 
    }
   else if(Ejt<0.80 || AccZ>0){  //We can remove this code in oredr get the ignition continous.Holah
    digitalWrite(v,LOW);
     Serial.println("Haan");
    } 
 */
  float altitude1 = myPressure.readAltitude();
  Serial.print("Altitude(m):");
  Serial.print(altitude, 2);
  currentTime = millis(); 
  previousTime = currentTime; 
  one=Serial.println(altitude);
  
 //delay(375);
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;
  float altitude2 = myPressure.readAltitude();
  two=Serial.println(altitude);


//  float altitude = myPressure.readAltitudeFt();
 // Serial.print(" Altitude(ft):");
  //
  Serial.print(altitude, 2);

  float pressure = myPressure.readPressure();
  Serial.print("Pressure(Pa):");
  Serial.print(pressure, 2);

  //float temperature = myPressure.readTemp();
  //Serial.print(" Temp(c):");
  //Serial.print(temperature, 2);

  float temperature = myPressure.readTempF();
  Serial.print(" Temp(f):");
  Serial.print(temperature, 2);
  

  Serial.println();
  

    if(Ejt>=0.80 || AccZ<0){
    Serial.println("Parachute");
    digitalWrite(v,HIGH);
    Serial.println("ukkeyyyy");
 
    }
   else if(Ejt<0.80 || AccZ>0 || two-one>0){  //We can remove this code in oredr get the ignition continous.Holah
    digitalWrite(v,LOW);
     Serial.println("Haan");
    } 
   else if(two-one<0){
    digitalWrite(v,HIGH);
    Serial.print("Delta y:");
    Serial.println(two-one);
    }
  // Complementary filter - combine acceleromter and gyro angle values
/*  
 roll = 0.96 * GyroX + 0.04 * AccX;
  pitch = 0.96 * GyroY + 0.04 * AccY;
  Serial.print(roll);
  Serial.print("|||");
  Serial.println(pitch);
  delay(1000);*/
  
  //Print the values on the serial monitor
/* Serial.print("AccX");Serial.print(AccX);
  Serial.print("||");
  Serial.print("AccY");Serial.print(AccY);
  Serial.print("||");
  Serial.print("AccZ");Serial.println(AccZ);
  delay(100);
    /*Serial.print("GyroX");Serial.print(GyroX);
  Serial.print("||");
  Serial.print("GyroY");Serial.print(GyroY);
  Serial.print("||");
  Serial.print("GyroZ");Serial.println(GyroZ);
  delay(100);
  Serial.print("roll");Serial.print(roll);
  Serial.print("||");
  Serial.print("pitch");Serial.print(pitch);
  Serial.print("||");
  Serial.print("yaw");Serial.println(yaw);
  delay(1000);
  /*Serial.print("accAngleX");Serial.print(accAngleX);
  Serial.print("||");
  Serial.print("accAngleY");Serial.print(accAngleY);
  Serial.print("||");
  Serial.print("gyroAngleX");Serial.print(gyroAngleX);
  Serial.print("||");
  Serial.print("gyroAngleY");Serial.println(gyroAngleY);
  delay(1000) ;*/


 
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {    
    myFile.print("Ejt: ");
    myFile.println(Ejt);
    myFile.print("Height: ");
    myFile.println(altitude);   
    myFile.print("Temperature: ");
     myFile.println(temperature);
    myFile.close(); // close the file
  }
  // if the file didn't open, print an error:
  else {
    Serial.println("error opening test.txt");
  }
  
 
}
void calculate_IMU_error() {
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  delay(2000);
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);

}
