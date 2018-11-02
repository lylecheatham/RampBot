#include "Motor.hpp"
#include "InterruptDisable.h"

Motor::Motor(MotorNum m, bool PID_enable)
{
	if(m == MotorA)
	{
		pwm_pin = M_PWMA;
		in1_pin = M_AIN1;
		in2_pin = M_AIN2;
		enc = std::make_unique<Encoder>(M_AENC1, M_AENC2);
	}
	else
	{
		pwm_pin = M_PWMB;
		in1_pin = M_AIN1;
		in2_pin = M_AIN2;
		enc = std::make_unique<Encoder>(M_BENC1, M_BENC2);
	}

	enc->write(0);
    previous_encoder_value = enc->read();

	// Begin the interval timer
	//intTime.begin(PID_control, 5000);
    {
        InterruptDisable d();
        interrupt_list.push_back(this);
    }


}

Motor::~Motor()
{
    InterruptDisable d();
    interrupt_list.remove(this);
}

uint8_t Motor::set_speed(float speed)
{
	return true;
}

float Motor::get_speed()
{
	return 0;
}

int32_t Motor::get_count()
{
	return enc->read();
}

void Motor::PID_control() {

	// Add error
	double error = target_speed - previous_speed;

	//Proporsional Value
	double p_out = k_term*error;

	//Integral Value
	double i_out = i_term*error/freq;

	//Derivative Value
	double d_out = d_term*error*freq;

	pwm_val = p_out + i_out + d_out;
}

void Motor::control_interrupt(){
    for (Motor* motor : interrupt_list){
        motor->PID_control();
    }
}
