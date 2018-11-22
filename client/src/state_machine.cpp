/* state_machine.cpp
 * MTE 380 Design Project
 * Original Author: Conner Currie
 * Creation Date: 11/12/2018
 *
 * Class containing functions for traversing the course
 */

#include "state_machine.hpp"
#include "All_Movements.hpp"


elapsedMillis time;

/* function: state_machine()
 *
 *
 *
 */
state_machine::state_machine() {
    robot.init();
}


/* function: ~state_machine()
 *
 *
 */
state_machine::~state_machine() {
}

inline void state_machine::get_dist(int32_t& dist) {
    dist = robot.swivel.sensor.ping_cm();
    dist = dist == 0 ? 300 : dist;
}


/* function: start()
 *
 * 	 	Function for initially finding the position of the robot on the course
 *
 *
 */
void state_machine::start() {
    while (1) {
        // First wait for keypress
        Serial.print("Waiting for keypress");

        int8_t character = get_char();
        while (character != 'a' && character != 'd' && character != 'w' && character != 's') {
            delayMicroseconds(100000);
            Serial.print(".");
            // Serial.println(servo->sensor.ping_cm());
            // imu->get_yaw();
            character = get_char();
        }

        // Initialize left and right turns
        TurnAngle turnL(-90);
        TurnAngle turnR(90);

        //#########################################################################
        // PHASE 1 - ASSUME ROBOT IS FACING THE BACK BOUNDARY WALL
        //#########################################################################

        // Get distance to the back wall and feed that into DriveDistance
        // Note - subtract the length of the robot from this (18cm plus 5cm tol)
        DriveDistance fwd_1(robot.swivel.sensor.ping_cm() - 25, 20);
        execute(fwd_1);

        // Take a right turn
        execute(turnR);

        // Get distance to the side wall and feed that into DriveDistance
        // Note - subtract the distance to the ramp from this (FIND VALUE)
        DriveDistance fwd_2(robot.swivel.sensor.ping_cm() - 25, 20);
        execute(fwd_2);

        // Take a left turn
        execute(turnL);

        // Carry out ramp movement
        RampMovement ramp;
        execute(ramp);

        // Take a left turn
        execute(turnL);

        // Move past the ramp to avoid pinging it
        // Assumed 20cm - FIND THIS LATER
        DriveDistance fwd_3(20, 20);
        execute(fwd_3);

        // Carry out post detection algorithm - first attempt:

        // NOT IMPLEMENTED YET
        // FindPost first_pass();

        // Touch  the post
        DriveDistance fwd_4(20, 20);
        execute(fwd_4);

        //#########################################################################
        // PHASE 2 - THE RETURN
        //#########################################################################

        // Turn to face the back
        // Take a left turn
        execute(turnL);
        // Take a left turn
        execute(turnL);

        // Get distance to the back wall and feed that into DriveDistance
        // Note - subtract the length of the robot from this (18cm plus 5cm tol)
        DriveDistance fwd_5(robot.swivel.sensor.ping_cm() - 25, 20);
        execute(fwd_5);

        // Take a left turn
        execute(turnL);

        // Get distance to the side wall and feed that into DriveDistance
        // Note - subtract the distance to the ramp from this (FIND VALUE)
        DriveDistance fwd_6(robot.swivel.sensor.ping_cm() - 25, 20);
        execute(fwd_6);

        // Take a right turn
        execute(turnR);

        // Carry out ramp movement
        execute(ramp);

        // Take a right turn
        execute(turnR);

        // Carry out first distances in inverted order
        execute(fwd_2);
        execute(turnL);
        execute(fwd_1);

        // Finish
    }
}


/* function: execute
 * 		Executes a given movement
 */
Status state_machine::execute(Movement& m) {
    return m.run(robot);
}

/* function: get_char()
 *
 */
int8_t state_machine::get_char() {
    if (Serial.available() > 0) return Serial.read();
    return -1;
}


/* function: stop()
 *
 */
void state_machine::stop() {
    robot.mA.set_speed(0);
    robot.mB.set_speed(0);
}
