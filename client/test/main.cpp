/************************************************************
MPU9250_DMP_Quaternion
 Quaternion example for MPU-9250 DMP Arduino Library
Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library
The MPU-9250's digital motion processor (DMP) can calculate
four unit quaternions, which can be used to represent the
rotation of an object.
This exmaple demonstrates how to configure the DMP to
calculate quaternions, and prints them out to the serial
monitor. It also calculates pitch, roll, and yaw from those
values.
Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0
Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/

#include <Arduino.h>
#include "IMU.hpp"

#define SerialPort SerialUSB

void setup() {
    SerialPort.begin(115200);
    while (!Serial)
        ;
    pinMode(STD_LED, OUTPUT);
    digitalWrite(STD_LED, 1);

    IMU imu;

    imu.imu->setCompassSampleRate(100);

    while (1) {
        imu.imu->computeCompassHeading();
        Serial.println(imu.imu->heading);
        delay(10);
    }
}

void loop() {}
