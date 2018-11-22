#pragma once

#include "Angle.hpp"
#include "IMU.hpp"
#include "Motor.hpp"
#include "ultrasonicSwivel.h"

class Robot {
public:
    Robot();

    bool init();

    Motor mA;
    Motor mB;
    UltraSonicSwivel swivel;
    IMU imu;

    std::array<Motor*, 2> motors;

    Angle target_angle;
};
