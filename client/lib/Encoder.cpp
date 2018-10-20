#include "Encoder.hpp"

EncoderInterface::EncoderInterface() :
{
	enc_timer.begin(update_speed, 1000);
}

int32 EncoderInterface::get_count()
{
	if(m == MotorA)
		return count[0];
	else
		return count[1];

}

int32 EncoderInterface::get_speed()
{
	if(m == MotorA)
		return speed[0];
	else
		return speed[1];

}
