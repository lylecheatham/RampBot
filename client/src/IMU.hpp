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

    float get_pitch() const;

    Angle get_pitch_abs() const;
    Angle get_yaw_abs() const;
    Angle get_roll_abs() const;
	
	float get_accel_y();

    Angle get_pitch_lp() const;
    Angle get_yaw_lp() const;
    Angle get_roll_lp() const;

    void compensate_pitch(const float coefficient, const Angle angle);
    void compensate_yaw(const float coefficient, const Angle angle);
    void compensate_roll(const float coefficient, const Angle angle);

    void stabilize();

private:
    static void complement(Angle &to_complement, float coefficient, const Angle complement_with);
    static float compensate_float(const Angle to_compensate, float coefficient, const Angle compensate_with);

    Angle get_compass_abs() const;
    void compensate_compass(const float coefficient, const Angle angle);
    void complementary_compass_filter();

    float pitch_compensation;
    float yaw_compensation;
    float roll_compensation;
    float compass_compensation;

    static constexpr float pitch_lp_constant = 0.05;
    static constexpr float yaw_lp_constant = 0.05;
    static constexpr float roll_lp_constant = 0.05;

    Angle pitch_lp;
    Angle yaw_lp;
    Angle roll_lp;

    float pitch;
    float pitch_prev;

    static IMU* singleton;
    static MPU9250_DMP* imu;
    static void updateIMU();
};

#endif
