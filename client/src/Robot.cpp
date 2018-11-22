#include "Robot.hpp"
#include "constants.h"

Robot::Robot() : mA(MotorA, true), mB(MotorB, true), swivel(S_PULSE, U_PING, 1), imu(){
    motors[0] = &mA;
    motors[1] = &mB;
}

bool Robot::init(){
    imu.init();

    pinMode(M_STDBY, OUTPUT);
    digitalWrite(M_STDBY, 1);

    if (!Motor::init()) { Serial.println("interrupt init fail;"); }

    mA.set_speed(0);
    mB.set_speed(0);

    imu.stabilize();

    target_angle = imu.get_yaw_abs();

    return true;
}
