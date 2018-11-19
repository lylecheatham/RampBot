/* state_machine.cpp
 * MTE 380 Design Project
 * Original Author: Conner Currie
 * Creation Date: 11/12/2018
 *
 * Class containing functions for traversing the course
 */

#include "state_machine.hpp"
#include "DemoInterface.hpp"
#include "ultraSonicSwivel.h"
#include "All_Movements.hpp"


elapsedMillis time;

/* Setup the static variables */
// center the servo
int state_machine::servo_pos = 90;

Motor* state_machine::mA = nullptr;
Motor* state_machine::mB = nullptr;

std::string state_machine::error_string = "";

// Initialize swivel Variable
UltraSonicSwivel* state_machine::servo = nullptr;

// Initialize IMU
IMU* state_machine::imu = nullptr;

// Initialize the Ultrasonic Sensor - (pin,pin,max)
NewPing* state_machine::sonar = nullptr;

/* function: state_machine()
 *
 *
 *
 */
state_machine::state_machine() {
    mA = new Motor(MotorA, true);
    mB = new Motor(MotorB, true);
    servo = new UltraSonicSwivel(S_PULSE, U_PING, 1);
    sonar = new NewPing(U_PING, U_PING, 300);
    imu = new IMU();

    pinMode(M_STDBY, OUTPUT);
    digitalWrite(M_STDBY, 1);
    if (!Motor::intTime.begin(Motor::control_interrupt, 1000000 / Motor::freq)) {
        error_string.append("interrupt init fail;");
    }
    stop();
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
    dist = sonar->ping_cm();
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
        while (character != 'e' && character != 'r') {
            delayMicroseconds(100000);
            Serial.print(".");
			character = get_char();
        //    Serial.println(sonar->ping_cm());
        }
				//Serial.println(execute(turnL));
		TurnAngle turnL(-90, mA, mB, imu, 60);
		TurnAngle turnR(90, mA, mB, imu, 60);

		if(character == 'r')
			Serial.println(execute(turnR));
		else
			Serial.println(execute(turnL));


/*
 *        int32_t distance;
 *        get_dist(distance);
 *
 *        forward();
 *        // Drive to boundary
 *        while (distance > 60) {
 *            get_dist(distance);
 *        }
 *
 *        // Turn right
 *        stop();
 *        turn_right();
 *        forward();
 *
 *        get_dist(distance);
 *        // Drive to ramp
 *        while (distance > 20) {
 *            get_dist(distance);
 *        }
 *
 *        stop();
 *
 *        // Turn left
 *        turn_left();
 *
 *        // Reverse up ramp
 *        backward();
 *
 *        // Stop once off ramp
 *        get_dist(distance);
 *        while (distance < 300) {
 *            get_dist(distance);
 *        }
 *        while (distance > 20) {
 *            get_dist(distance);
 *        }
 *
 *        stop();
 *
 *        // Turn left
 *        turn_left();
 *
 *        // Turn servo towards wall
 *        servo->set_position(180);
 *
 *        // Drive until distance closer than 240cm found
 *        get_dist(distance);
 *        while (distance > 20) {
 *            get_dist(distance);
 *        }
 *        stop();
 *        servo->set_position(90);
 *
 *        // Turn right
 *        turn_right();
 *
 *        // Drive straight until distance small
 *        forward();
 *        get_dist(distance);
 *        while (distance > 10) {
 *            get_dist(distance);
 *        }
 *
 *        stop();
 */
    }
}


/* function: execute
 * 		Executes a given movement
 */
Status state_machine::execute(Movement &m)
{
	while((m.update() == ONGOING));
	return m.last_status;
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
