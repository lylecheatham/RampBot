#include "Motor.hpp"

Motor::Motor(MotorNum m)
{
	if(m == MotorA)
		enc = new Encoder(M_AENC1, M_AENC2);
	else
		enc = new Encoder(M_BENC1, M_BENC2);

	enc->write(0);
    previous_encoder_value = enc->read();

	// Begin the interval timer
	//intTime.begin(PID_control, 5000);
}

Motor::~Motor()
{
	delete enc;
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

void Motor::PID_control(double setpoint, double input_val) {

	// Add error
	double error = setpoint - input_val;

	//Proporsional Value
	double p_out = k_term*error;

	//Integral Value
	double i_out = i_term/freq;

	//Derivative Value
	double d_out = d_term*freq;

	pwm_val = p_out + i_out + d_out;
}
