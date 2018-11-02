/* DemoInterface.hpp
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

#ifndef DEMO_INTERFACE_H
#define DEMO_INTERFACE_H

#include "Arduino.h"
#include "constants.h"
#include "IntervalTimer.h"
#include "Motor.hpp"

#include <list>

#define DEBUG_PRINT
#define SPEED 1
#define NUM_CMD 8

enum Command {
	INV=0,		// Invalid
	FWD,		// Forward
	BWD,		// Backward
	LEFT,		// Left
	RIGHT,		// Right
	SRV_LFT,	// Servo left (ccw)
	SRV_RGT		// Servo right (cw)
};



class DemoInterface
{
	private:
		static Motor *mA;
	    static Motor *mB;

		IntervalTimer stop_timer;
		const uint32 delay = 500000;

		// Key mapping
		Command control_keys[26] =
		{
			LEFT, 		//a
			INV, 		//b
			INV,		//c
			RIGHT, 		//d
			SRV_LFT, 	//e
			INV, 		//f
			INV, 		//g
			INV, 		//h
			INV, 		//i
			INV, 		//j
			INV, 		//k
			INV, 		//l
			INV, 		//m
			INV, 		//n
			INV, 		//o
			INV, 		//p
			INV, 		//q
			SRV_RGT,	//r
			BWD, 		//s
			INV, 		//t
			INV, 		//u
			INV, 		//v
			FWD, 		//w
			INV, 		//x
			INV, 		//y
			INV  		//z

		};

		// Commands
		FunctionPointer commands[NUM_CMD] =
		{
			&error,
			&move_forward,
			&move_backward,
			&turn_left,
			&turn_right,
			&servo_left,
			&servo_right
		};

	public:
		DemoInterface();
		~DemoInterface();

		bool run_command(int8 key);

		static int8 get_char();

	private:
        static int servo_pos;

		static void error();
		static void move_forward();
		static void move_backward();
		static void turn_left();
		static void turn_right();
		static void servo_left();
		static void servo_right();
		static void stop();

};


#endif
