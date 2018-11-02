#include "Motor.hpp"

Motor::Motor(MotorNum m)
{
	// Set up the pins
	if(m == MotorA)
	{
		pwm_pin = M_PWMA;
		in1_pin = M_AIN1;
		in2_pin = M_AIN2;
		enc = new Encoder(M_AENC1, M_AENC2);
	}
	else
	{
		pwm_pin = M_PWMB;
		in1_pin = M_AIN1;
		in2_pin = M_AIN2;
		enc = new Encoder(M_BENC1, M_BENC2);
	}

	// Set pins
	pinMode(pwm_pin, OUTPUT);
	pinMode(in1_pin, OUTPUT);
	pinMode(in2_pin, OUTPUT);

	// Default the encoder
	enc->write(0);
    previous_encoder_value = enc->read();

	// Begin the interval timer
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

void Motor::PID_control() {



}
