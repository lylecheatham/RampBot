#include "Motor.hpp"

Motor::Motor(MotorNum m)
{
	if(m == MotorA)
		enc = new Encoder(M_AENC1, M_AENC2);
	else
		enc = new Encoder(M_BENC1, M_BENC2);

	enc->write(0);
    previous_encoder_value = enc->read();
}

Motor::~Motor()
{
	delete enc;
}

uint8 Motor::set_speed(float speed)
{
	return true;
}

float Motor::get_speed()
{
	return 0;
}

int32 Motor::get_count()
{
	return enc->read();
}

void Motor::PID_control() {



}
