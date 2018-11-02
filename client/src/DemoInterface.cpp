/* DemoInterface.cpp
 * MTE 380 Design Project
 * Original Author: Eric Murphy-Zaremba
 * Creation Date: Oct 30 /2018
 *
 * This class is used for the purpose of demonstrating the robot
 * functionality.
 *
 * Functional points:
 * 		- (member) Run a command mapped to a received character
 *		- (static) Receive a character over serial
 */

#include "DemoInterface.hpp"
#include "ultraSonicSwivel.h"

int DemoInterface::servo_pos = 90;

Motor* DemoInterface::mA = nullptr;
Motor* DemoInterface::mB = nullptr;

//Initialize swivel Variable
UltraSonicSwivel* DemoInterface::servo = nullptr;

DemoInterface::DemoInterface()
{
    mA = new Motor(MotorA, true);
    mB = new Motor(MotorB, true);
	servo = new UltraSonicSwivel(U_PING, S_PULSE, 1);

    pinMode(M_STDBY, OUTPUT);
    digitalWrite(M_STDBY, 1);
	//Motor::intTime.begin(Motor::control_interrupt, 1000000/Motor::freq);
	stop();
}

DemoInterface::~DemoInterface()
{
	delete mA;
	delete mB;
	delete servo;
}

bool DemoInterface::run_command(int8 key)
{
	if(key > 'z' || key < 'a')
		return false;

	(*commands[control_keys[key-'a']])();

	//Timer
	//stop_timer.begin(stop, delay);

	return true;
}

void DemoInterface::error()
{
#ifdef DEBUG_PRINT
	Serial.println("Wrong key");
#endif

}

void DemoInterface::move_forward()
{
#ifdef DEBUG_PRINT
	Serial.println("Moving forward");
#endif
	/*

	// CCW
	mA->set_speed(-SPEED);	

	// CW
	mB->set_speed(SPEED);

	*/

	// Temporarily just turn on working motor at low speed
	mA->set_speed(SPEED);
}

void DemoInterface::move_backward()
{
#ifdef DEBUG_PRINT
	Serial.println("Moving backward");
#endif
	/*
	// CCW
	mB->set_speed(-SPEED);	

	// CW
	mA->set_speed(SPEED);
	*/
	mA->set_speed(-SPEED);
}

void DemoInterface::stop()
{
#ifdef DEBUG_PRINT
	Serial.println("Stopping");
#endif

	// Put both motors in STOP
	mA->set_speed(0);
	mB->set_speed(0);
}

void DemoInterface::turn_left()
{
#ifdef DEBUG_PRINT
	Serial.println("Turning Left");
#endif

	// CW
	mA->set_speed(SPEED);
	mB->set_speed(SPEED);
}

void DemoInterface::turn_right()
{
#ifdef DEBUG_PRINT
	Serial.println("Turning right");
#endif

	// CCW
	mA->set_speed(-SPEED);
	mB->set_speed(-SPEED);
}

void DemoInterface::servo_left()
{
    int32_t current_pos = servo->get_position();
	servo->set_position(current_pos - 1);
}

void DemoInterface::servo_right()
{
    int32_t current_pos = servo->get_position();
	servo->set_position(current_pos + 1);
}



int8 DemoInterface::get_char()
{
	if(Serial.available() > 0)
		return Serial.read();
	return -1;
}
