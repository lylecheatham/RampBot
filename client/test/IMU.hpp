/* IMU.hpp
 * MTE 380 Design Project
 * Original Author(s): Conner Currie & Eric Murphy-Zaremba
 * Creation Date: Nov 14 /2018
 */

#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include "constants.h"
#include <SparkFunMPU9250-DMP.h>

class IMU {
public:
    IMU();
    ~IMU();

    float get_pitch();
    float get_yaw();
    float get_roll();
    static MPU9250_DMP* imu;

private:

    float pitch;
    float yaw;
    float roll;

    static void updateIMU();
};

#endif
