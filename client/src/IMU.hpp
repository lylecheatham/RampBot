/* IMU.hpp
 * MTE 380 Design Project
 * Original Author(s): Conner Currie & Eric Murphy-Zaremba
 * Creation Date: Nov 14 /2018
 */

#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include <SparkFunMPU9250-DMP.h>
#include "Angle.hpp"
#include "constants.h"

class IMU {
public:
    IMU();

    bool init();

    float get_pitch();

    Angle get_pitch_abs();
    Angle get_yaw_abs();
    Angle get_roll_abs();

    void compensate_pitch(float coefficient, Angle angle);
    void compensate_yaw(float coefficient, Angle angle);
    void compensate_roll(float coefficient, Angle angle);

    void stabilize();

private:
    float pitch_compensation;
    float yaw_compensation;
    float roll_compensation;


    float pitch;
    float pitch_prev;

    static IMU* singleton;
    static MPU9250_DMP* imu;
    static void updateIMU();
};

#endif
