/* IMU.cpp
 * MTE 380 Design Project
 * Original Author(s): Eric Murphy-Zaremba
 * Creation Date: Nov 14 /2018
 *
 *
 * Implements the IMU class
 */


#include "IMU.hpp"
#include "quaternionFilters.hpp"


// Init MPU9250 FIFO
MPU9250* IMU::MPU = nullptr;
QuaternionFilter IMU::filter = QuaternionFilter();

// Init imu values
char IMU::print_buf[200];

IMU::IMU() {
    MPU = new MPU9250(MPU9250_ADDRESS_AD0, Wire, I2Cclock);

    Wire.begin();

    // Set up interrupt
    pinMode(IMU_INT, INPUT);
    digitalWrite(IMU_INT, LOW);
    attachInterrupt(IMU_INT, read_IMU, RISING);

    // Test communication
    byte c = MPU->readByte(MPU9250_ADDRESS_AD0, WHO_AM_I_MPU9250);
    if (c != 0x71) {
        Serial.print("Unable to connect to IMU: 0x");
        Serial.println(c, HEX);
        Serial.flush();
        abort();
    }

#if SerialDebug
    Serial.println("MPU Up and running");
    // Start by performing self test and reporting values
    MPU->MPU9250SelfTest(MPU->selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(MPU->selfTest[0], 1);
    Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(MPU->selfTest[1], 1);
    Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(MPU->selfTest[2], 1);
    Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(MPU->selfTest[3], 1);
    Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(MPU->selfTest[4], 1);
    Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(MPU->selfTest[5], 1);
    Serial.println("% of factory value");
#endif

    // Calibration
    MPU->calibrateMPU9250(MPU->gyroBias, MPU->accelBias);
    MPU->initMPU9250();
    MPU->initAK8963(MPU->factoryMagCalibration);

    // Resolution
    MPU->getAres();
    MPU->getGres();
    MPU->getMres();
//    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
#if SerialDebug
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(MPU->magBias[0]);
    Serial.println(MPU->magBias[1]);
    Serial.println(MPU->magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(MPU->magScale[0]);
    Serial.println(MPU->magScale[1]);
    Serial.println(MPU->magScale[2]);

    Serial.println("Magnetometer:");
    Serial.print("X-Axis sensitivity adjustment value ");
    Serial.println(MPU->factoryMagCalibration[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value ");
    Serial.println(MPU->factoryMagCalibration[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value ");
    Serial.println(MPU->factoryMagCalibration[2], 2);
#endif
}

IMU::~IMU() {
    delete MPU;
}

void IMU::print_values() {
#if SerialDebug
    // Serial.println(digitalRead(IMU_INT));
    /*
Serial.print("Yaw, Pitch, Roll: ");
Serial.print((170 + MPU->yaw)/0.675, 2);
Serial.print(", ");
Serial.print(MPU->pitch, 2);
Serial.print(", ");
Serial.println(MPU->roll, 2);

Serial.print("rate = ");
Serial.print((float)MPU->sumCount / MPU->sum, 2);
Serial.println(" Hz");*/
#endif
}

void IMU::read_IMU() {
    // Set up LED
    static bool LED_state = false;
    digitalWrite(13, LED_state);
    LED_state = !LED_state;
    // Serial.println(".");
    // MPU->readByte(MPU9250_ADDRESS_AD0, INT_STATUS); // Clear
    MPU->readAccelData(MPU->accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    MPU->ax = (float)MPU->accelCount[0] * MPU->aRes;  // - MPU->accelBias[0];
    MPU->ay = (float)MPU->accelCount[1] * MPU->aRes;  // - MPU->accelBias[1];
    MPU->az = (float)MPU->accelCount[2] * MPU->aRes;  // - MPU->accelBias[2];

    MPU->readGyroData(MPU->gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    MPU->gx = (float)MPU->gyroCount[0] * MPU->gRes;
    MPU->gy = (float)MPU->gyroCount[1] * MPU->gRes;
    MPU->gz = (float)MPU->gyroCount[2] * MPU->gRes;

    MPU->readMagData(MPU->magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    MPU->mx = (float)MPU->magCount[0] * MPU->mRes * MPU->factoryMagCalibration[0] - MPU->magBias[0];
    MPU->my = (float)MPU->magCount[1] * MPU->mRes * MPU->factoryMagCalibration[1] - MPU->magBias[1];
    MPU->mz = (float)MPU->magCount[2] * MPU->mRes * MPU->factoryMagCalibration[2] - MPU->magBias[2];

    // Must be called before updating quaternions!

    MPU->updateTime();

    // Mahony Algorithm
    filter.mahonyUpdate(MPU->ax, MPU->ay, MPU->az, MPU->gx * DEG_TO_RAD, MPU->gy * DEG_TO_RAD, MPU->gz * DEG_TO_RAD, MPU->my, MPU->mx, MPU->mz, MPU->deltat);

    if (!AHRS) {
        MPU->delt_t = millis() - MPU->count;
        if (MPU->delt_t > 100) {
            MPU->count = millis();
            digitalWrite(STD_LED, !digitalRead(STD_LED));  // toggle led
        }                                                  // if (MPU->delt_t > 500)
    }                                                      // if (!AHRS)
    else {
        // Serial print and/or display at 0.5 s rate independent of data rates
        MPU->delt_t = millis() - MPU->count;

        // update LCD once per half-second independent of read rate
        if (MPU->delt_t > 100) {
            // Get Pitch, Yaw and Roll fromQuaternions
            const auto q = filter.getQ();
            MPU->yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),
                             pow(q[0], 2) + pow(q[1], 2) - pow(q[2], 2) - pow(q[3], 2));
            MPU->pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
            MPU->roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
                              pow(q[0], 2) - pow(q[1], 2) - pow(q[2], 2) + pow(q[3], 2));
            MPU->pitch *= RAD_TO_DEG;
            MPU->yaw *= RAD_TO_DEG;
            MPU->roll *= RAD_TO_DEG;

            MPU->count = millis();
            MPU->sumCount = 0;
            MPU->sum = 0;
        }  // if (MPU->delt_t > 500)
    }      // if (AHRS)
}
