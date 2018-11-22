/* state_machine.cpp
 * MTE 380 Design Project
 * Original Author: Conner Currie
 * Creation Date: 11/12/2018
 *
 * Class containing functions for traversing the course
 */

#include "state_machine.hpp"
#include "All_Movements.hpp"
#include "DemoInterface.hpp"
#include "ultraSonicSwivel.h"


elapsedMillis time;

/* Setup the static variables */
// center the servo
int state_machine::servo_pos = 90;

Motor* state_machine::mA = nullptr;
Motor* state_machine::mB = nullptr;

std::string state_machine::error_string = "";

// Initialize US and Servo Variable
UltraSonicSwivel* state_machine::servo = nullptr;

// Initialize IMU
IMU* state_machine::imu = nullptr;

/* function: state_machine()
 *
 *
 *
 */
state_machine::state_machine() {
    mA = new Motor(MotorA, true);
    mB = new Motor(MotorB, true);

    // Initialize Servo/US
    servo = new UltraSonicSwivel(S_PULSE, U_PING, 1);
    imu = new IMU();
    imu->init();


    pinMode(M_STDBY, OUTPUT);
    digitalWrite(M_STDBY, 1);
    if (!Motor::init()) { error_string.append("interrupt init fail;"); }
    stop();


    imu->stabilize();  // wait for the imu value to stabilize
}


/* function: ~state_machine()
 *
 *
 */
state_machine::~state_machine() {
    delete mA;
    delete mB;
    delete servo;
    // delete imu;
}

inline void state_machine::get_dist(int32_t& dist) {
    dist = servo->sensor.ping_cm();
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

		DriveToPost post(60, mA, mB, servo, imu, 20);
		execute(post);


       /* // Initialize left and right turns*/
        //TurnAngle turnL(-90, mA, mB, imu);
        //TurnAngle turnR(90, mA, mB, imu);

        ////#########################################################################
        //// PHASE 1 - ASSUME ROBOT IS FACING THE BACK BOUNDARY WALL
        ////#########################################################################

        //// Get distance to the back wall and feed that into DriveDistance
        //// Note - subtract the length of the robot from this (18cm plus 5cm tol)
        //DriveDistance fwd_1(servo->sensor.ping_cm() - 25, mA, mB, servo, imu, 20);
        //execute(fwd_1);

        //// Take a right turn
        //execute(turnR);

        //// Get distance to the side wall and feed that into DriveDistance
        //// Note - subtract the distance to the ramp from this (FIND VALUE)
        //DriveDistance fwd_2(servo->sensor.ping_cm() - 25, mA, mB, servo, imu, 20);
        //execute(fwd_2);

        //// Take a left turn
        //execute(turnL);

        //// Carry out ramp movement
        //RampMovement ramp(mA, mB, imu);
        //execute(ramp);

        //// Take a left turn
        //execute(turnL);

        //// Move past the ramp to avoid pinging it
        //// Assumed 20cm - FIND THIS LATER
        //DriveDistance fwd_3(20, mA, mB, servo, imu, 20);
        //execute(fwd_3);

        //// Carry out post detection algorithm - first attempt:

        //// NOT IMPLEMENTED YET
        //// FindPost first_pass();

        //// Touch  the post
        //DriveDistance fwd_4(20, mA, mB, servo, imu, 20);
        //execute(fwd_4);

        ////#########################################################################
        //// PHASE 2 - THE RETURN
        ////#########################################################################

        //// Turn to face the back
        //// Take a left turn
        //execute(turnL);
        //// Take a left turn
        //execute(turnL);

        //// Get distance to the back wall and feed that into DriveDistance
        //// Note - subtract the length of the robot from this (18cm plus 5cm tol)
        //DriveDistance fwd_5(servo->sensor.ping_cm() - 25, mA, mB, servo, imu, 20);
        //execute(fwd_5);

        //// Take a left turn
        //execute(turnL);

        //// Get distance to the side wall and feed that into DriveDistance
        //// Note - subtract the distance to the ramp from this (FIND VALUE)
        //DriveDistance fwd_6(servo->sensor.ping_cm() - 25, mA, mB, servo, imu, 20);
        //execute(fwd_6);

        //// Take a right turn
        //execute(turnR);

        //// Carry out ramp movement
        //execute(ramp);

        //// Take a right turn
        //execute(turnR);

        //// Carry out first distances in inverted order
        //execute(fwd_2);
        //execute(turnL);
        //execute(fwd_1);

        // Finish
    }
}


/* function: execute
 * 		Executes a given movement
 */
Status state_machine::execute(Movement& m) {
    return m.run();
}

/* function: get_char()
 *
 */
int8_t state_machine::get_char() {
    if (Serial.available() > 0) return Serial.read();
    return -1;
}

/* function: turn_left()
 *
 */
void state_machine::turn_left() {
    /*
     *int32_t curr_bearing = imu->get_orientation();
     *mA->set_speed(speed);
     *while (imu->get_orientation() != curr_bearing + 90);
     *stop();
     */
    time = 0;
    stop();
    mA->set_speed(speed);
    while (time < 2000)
        ;
    stop();
}

/* function: turn_right()
 *
 */
void state_machine::turn_right() {
    /*
     *int32_t curr_bearing = imu->get_orientation();
     *mB->set_speed(speed);
     *while (imu->get_orientation() != curr_bearing - 90);
     *stop();
     */
}

/* function: stop()
 *
 */
void state_machine::stop() {
    mA->set_speed(0);
    mB->set_speed(0);
}

/* function: forward()
 *
 */
void state_machine::forward() {
    mA->set_speed(speed);
    mB->set_speed(speed);
}


/* function: backward()
 *
 */
void state_machine::backward() {
    mA->set_speed(-speed);
    mB->set_speed(-speed);
}
