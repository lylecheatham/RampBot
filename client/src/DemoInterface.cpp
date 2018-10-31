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


DemoInterface::DemoInterface() 
{
	stop();
		
	// Set PWM to the desired frequency
	digitalWrite(M_PWMA, SPEED);
	digitalWrite(M_PWMB, SPEED);	
}

bool DemoInterface::run_command(int8 key)
{
	if(key > 'z' || key < 'a')
		return false;
	
	(*commands[control_keys[key-'a']])();	
	
	//Timer
	stop_timer.begin(stop, delay);

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

	// CCW
	digitalWrite(M_AIN1, 0);
	digitalWrite(M_AIN2, 1);

	// CW
	digitalWrite(M_BIN1, 1);
	digitalWrite(M_BIN2, 0);
}

void DemoInterface::move_backward()
{
#ifdef DEBUG_PRINT
	Serial.println("Moving backward");
#endif

	// CW
	digitalWrite(M_AIN1, 1);
	digitalWrite(M_AIN2, 0);

	// CCW
	digitalWrite(M_BIN1, 0);
	digitalWrite(M_BIN2, 1);
}

void DemoInterface::stop()
{
#ifdef DEBUG_PRINT
	Serial.println("Stopping");
#endif

	// Put both motors in STOP
	digitalWrite(M_AIN1, 0);
	digitalWrite(M_AIN2, 0);
	digitalWrite(M_BIN1, 0);
	digitalWrite(M_BIN2, 0);

}

void DemoInterface::turn_left()
{
#ifdef DEBUG_PRINT
	Serial.println("Turning Left");
#endif

	// CW
	digitalWrite(M_AIN1, 1);
	digitalWrite(M_AIN2, 0);
	digitalWrite(M_BIN1, 1);
	digitalWrite(M_BIN2, 0);	
}

void DemoInterface::turn_right()
{
#ifdef DEBUG_PRINT
	Serial.println("Turning right");
#endif

	// CCW
	digitalWrite(M_AIN1, 0);
	digitalWrite(M_AIN2, 1);
	digitalWrite(M_BIN1, 0);
	digitalWrite(M_BIN2, 1);
}

void DemoInterface::servo_left()
{}

void DemoInterface::servo_right()
{}

int8 DemoInterface::get_char()
{
	if(Serial.available() > 0)
		return Serial.read();
	return -1;
}
