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
#include "IntervalTimer.h"
#include "Motor.hpp"
#include "constants.h"
#include "ultraSonicSwivel.h"

#include <list>

#define DEBUG_PRINT

enum Command {
    INV = 0,  // Invalid
    STOP,     // Stop
    FWD,      // Forward
    BWD,      // Backward
    LEFT,     // Left
    RIGHT,    // Right
    SRV_LFT,  // Servo left (ccw)
    SRV_RGT,  // Servo right (cw)
    PRN_ERR,  // Get Error
    NUM_CMD
};


class DemoInterface {
private:
    static Motor *mA;
    static Motor *mB;
    static UltraSonicSwivel *servo;
    static int servo_pos;
    static std::string error_string;

    IntervalTimer stop_timer;
    const uint32_t delay = 500000;

    static int32_t speedA, speedB;

    // Key mapping
    Command control_keys[26] = {
        LEFT,     // a
        INV,      // b
        INV,      // c
        RIGHT,    // d
        SRV_LFT,  // e
        INV,      // f
        INV,      // g
        INV,      // h
        INV,      // i
        INV,      // j
        INV,      // k
        INV,      // l
        INV,      // m
        INV,      // n
        INV,      // o
        INV,      // p
        PRN_ERR,  // q
        SRV_RGT,  // r
        BWD,      // s
        INV,      // t
        INV,      // u
        INV,      // v
        FWD,      // w
        STOP,     // INV, 		//x
        INV,      // y
        INV       // z

    };

    typedef void (*command_fn_ptr)();

    // Commands
    command_fn_ptr commands[NUM_CMD] = {&error, &stop, &move_forward, &move_backward, &turn_left, &turn_right, &servo_left, &servo_right, &print_error};

public:
    DemoInterface();
    ~DemoInterface();

    bool run_command(int8_t key);

    static int8_t get_char();

private:
    void standby();
    static void update_speeds();

    static void error();
    static void move_forward();
    static void move_backward();
    static void turn_left();
    static void turn_right();
    static void servo_left();
    static void servo_right();
    static void stop();
    static void print_error();
};


#endif
