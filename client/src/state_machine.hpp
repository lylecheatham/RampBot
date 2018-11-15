/* state_machine.hpp
 * MTE 380 Design Project
 * Original Author: Conner Currie
 * Creation Date: 11/12/2018
 *
 * Class containing functions for traversing the course
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "Arduino.h"
#include "IntervalTimer.h"
#include "Motor.hpp"
#include "constants.h"
#include "ultraSonicSwivel.h"
#include "IMU.hpp"


#include <list>

#define DEBUG_PRINT

class state_machine {
private:
    static Motor *mA;
    static Motor *mB;
    static UltraSonicSwivel *servo;
    static int servo_pos;
    static std::string error_string;
	static IMU *imu;

    //State tracking variables
    static int32_t start_pos[2];
    static int32_t pole_pos[2];
    static int32_t cur_dist;
    static int32_t state;

    IntervalTimer stop_timer;
    const uint32_t delay = 500000;

    static int32_t speedA, speedB;

public:
    state_machine();
    ~state_machine();

    bool run_command(int8_t key);

    static int8_t get_char();

private:
    void standby();
    static void stop();
    void state_update();
    static void ramp_pos_1();
    static void ramp();
    static void pole_id();
    static void ramp_pos_2();
    static void base_return();
    static void update_speeds();

};


#endif
