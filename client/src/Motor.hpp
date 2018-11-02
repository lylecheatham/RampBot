#ifndef MOTOR_HPP 
#define MOTOR_HPP

#include "constants.h"
#include <Encoder.h>
#include <IntervalTimer.h>
#include <list>
#include <memory>

#define CPR_S 10.28 //counts_per_rev*60s

class Motor
{
    std::unique_ptr<Encoder> enc;
	IntervalTimer intTime;
	int32_t pwm_pin, in1_pin, in2_pin;

	static const int16_t max_speed = 600; //[rpm]

public:
    Motor(MotorNum m, bool PID_enable = true);
    ~Motor();

    void    set_speed(int32_t speed);
    int32_t get_speed();
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
    float	previous_speed;

private:
    void PID_control(double setpoint, double input_val);
    void PID_control();

    static void control_interrupt();
    static std::list<Motor*> interrupt_list;

};

#endif
