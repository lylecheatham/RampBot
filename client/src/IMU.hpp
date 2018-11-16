/* IMU.hpp
 * MTE 380 Design Project
 * Original Author(s): Eric Murphy-Zaremba
 * Creation Date: Nov 14 /2018
 */

#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include "MPU9250.hpp"
#include "constants.h"

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging
#define I2Cclock 400000

class IMU {
public:
    IMU();
    ~IMU();

    int32_t get_yaw();
    int32_t get_pitch();
    int32_t get_roll();

    void print_values();

private:
    /* Local Members */
    // float yaw_os, pitch_os, roll_os;

    // void zero();

    /* Static declarations*/
    static MPU9250 *MPU;         // Note that the SDR sets sampling to 200Hz currently
    static char print_buf[200];  // For debug and setup

    static void read_IMU();
};


#endif
