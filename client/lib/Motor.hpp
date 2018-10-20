#ifndef MOTOR_HPP 
#define MOTOR_HPP

#include "Encoder.hpp"

class Motor
{
	Encoder enc;

	public:
		Motor(int pwm_pin, int in1_pin, int in2_pin);
		~Motor();

		bool set_speed(float speed);
		float get_speed();

	private:


}

#endif
