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
#include "Movement.hpp"


#include <list>

#define DEBUG_PRINT

class state_machine {
private:
    // Static objects
    static Motor *mA;
    static Motor *mB;
    static UltraSonicSwivel *servo;
    static int servo_pos;
    static std::string error_string;
    static IMU *imu;

    // State tracking variables
    static int32_t start_pos[2];
    static int32_t pole_pos[2];
    static int32_t cur_dist;
    static int32_t state;

    // Speed to run motors at
    const static int32_t speed = 75;

public:
    state_machine();
    ~state_machine();

    void start();

private:
	Status execute(Movement &m);
    int8_t get_char();
    void standby();
    void turn_left();
    void turn_right();
    void stop();
    void forward();
    void backward();
    void get_dist(int32_t &dist);
};


#endif
