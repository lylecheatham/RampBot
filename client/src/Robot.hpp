#pragma once

#include "IMU.hpp"
#include "Motor.hpp"
#include "ultrasonicSwivel.h"
#include "Angle.hpp"

class Robot{
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

