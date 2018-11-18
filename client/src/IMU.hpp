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

    float get_pitch_abs();
    float get_yaw_abs();
    float get_roll_abs();

private:
    static MPU9250_DMP* imu;

    static float pitch;
    static float pitch_prev;
    static float yaw;
    static float yaw_prev;
    static float roll;
    static float roll_prev;

    static void updateIMU();
};

#endif
