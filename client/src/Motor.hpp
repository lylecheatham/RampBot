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

		uint8   set_speed(float32 speed);
		float32 get_speed();
		int32   get_count();


};

#endif
