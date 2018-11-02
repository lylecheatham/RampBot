#ifndef MOTOR_HPP 
#define MOTOR_HPP

#include "constants.h"
#include <Encoder.h>
#include <IntervalTimer.h>
#include <list>
#include <memory>

class Motor
{
    std::unique_ptr<Encoder> enc;
	IntervalTimer intTime;
	int32_t pwm_pin, in1_pin, in2_pin;


public:
    Motor(MotorNum m, bool PID_enable = true);
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

	int32_t freq;

    float pwm_val;

    int32_t previous_encoder_value;
    int32_t previous_speed;

private:
    void PID_control(double setpoint, double input_val);
    void PID_control();

    static void control_interrupt();
    static std::list<Motor*> interrupt_list;

};

#endif
