#ifndef MOTOR_HPP 
#define MOTOR_HPP

#include "constants.h"
#include <Encoder.h>
#include <IntervalTimer.h>

class Motor
{
	Encoder* enc;
	IntervalTimer intTime;

public:
    Motor(MotorNum m);
    ~Motor();

    uint8_t set_speed(float speed);
    float get_speed();
    int32_t get_count();

    float target_speed;

    int32_t i_counter;
    int32_t i_max;

    float d_term;
    float k_term;
    float i_term;

    int s_val;

    int32_t previous_encoder_value;
    int32_t previous_speed;

private:
    void PID_control(double in_vol);
};

#endif
