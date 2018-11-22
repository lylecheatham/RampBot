/* IMU.cpp
 * MTE 380 Design Project
 * Original Author(s): Conner Currie & Eric Murphy-Zaremba
 * Creation Date: Nov 14 /2018
 *
 *
 * Implements the IMU class
 */


#include "IMU.hpp"

// Init MPU9250
MPU9250_DMP* IMU::imu = nullptr;

IMU::IMU() {
    imu = new MPU9250_DMP();

    // Call imu.begin() to verify communication and initialize
    if (imu->begin() != INV_SUCCESS)
    {
        while (1)
        {
        Serial.println("Unable to communicate with MPU-9250");
        Serial.println("Check connections, and try again.");
        Serial.println();
        delay(5000);
        }
    }

    imu->dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              200); // Set DMP FIFO rate to 10 Hz
    // DMP_FEATURE_LP_QUAT can also be used. It uses the
    // accelerometer in low-power mode to estimate quat's.
    // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

    // Set up interrupt
    pinMode(IMU_INT, INPUT);
    digitalWrite(IMU_INT, LOW);
    attachInterrupt(IMU_INT, updateIMU, RISING);
}

IMU::~IMU() {
    delete imu;
}

float IMU::get_pitch() {

    return imu->pitch;
}

float IMU::get_yaw() {

    return imu->yaw;
}

float IMU::get_roll() {

    return imu->roll;
}

void IMU::updateIMU()
{
  // Check for new data in the FIFO
  if ( imu->fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu->dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu->computeEulerAngles();
    }
    imu->updateCompass();
  }
}
