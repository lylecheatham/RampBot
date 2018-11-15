/* state_machine.cpp
 * MTE 380 Design Project
 * Original Author: Conner Currie
 * Creation Date: 11/12/2018
 *
 * Class containing functions for traversing the course
 */

#include "state_machine.hpp"
#include "ultraSonicSwivel.h"
#include "DemoInterface.hpp"


//center the servo
int state_machine::servo_pos = 90;

Motor* state_machine::mA = nullptr;
Motor* state_machine::mB = nullptr;

int32_t state_machine::speedA = 0;
int32_t state_machine::speedB = 0;

std::string state_machine::error_string = "";

// Initialize swivel Variable
UltraSonicSwivel* state_machine::servo = nullptr;

// Initialize IMU
IMU* state_machine::imu = nullptr;

//Initialize the Ultrasonic Sensor - (pin,pin,max)
NewPing sonar(U_PING, U_PING, 300);


state_machine::state_machine() {
    //mA = new Motor(MotorA, true);
    //mB = new Motor(MotorB, true);
    //servo = new UltraSonicSwivel(S_PULSE, U_PING, 1);
    
	imu = new IMU();
	
    //pinMode(M_STDBY, OUTPUT);
    //if (!Motor::intTime.begin(Motor::control_interrupt, 1000000 / Motor::freq)) {
    //    error_string.append("interrupt init fail;");
    //}
    //stop();
}



state_machine::~state_machine() {
    delete mA;
    delete mB;
    delete servo;
	delete imu;
}


//Function for initially finding the position of the robot on the course
void state_machine::state_update(){
    //Main loop
    switch(state) { 
    case 0: //Drive towards back wall
        //statements 
        break; 
    case 1: //Drive towards ramp
        //statements
        break; 
    case 2: //Drive up and down ramp
        //statements
        break; 
    case 3: //Identify the post
        //statements
        break; 
    case 4: //Drive towards the post
        //statements
        break; 
    case 5: //Drive towards back wall
        //statements
        break; 
    case 6: //Drive towards ramp
        //statements
        break; 
    case 7: //Drive up and down ramp
        //statements
    case 8: //Drive towards starting location
        //statements
        break; 
}
    
}

void state_machine::ramp_pos_2(){
    //1. Drive towards the ramp 
}

void state_machine::ramp(){
    //1. Drive towards the ramp 
}

void state_machine::pole_id(){
    
}



void state_machine::base_return(){

}

void state_machine::update_speeds() {
    if (speedA > 150) speedA = 150;
    if (speedA < -150) speedA = -150;
    if (speedB > 150) speedB = 150;
    if (speedB < -150) speedB = -150;
    mA->set_speed(speedA);
    mB->set_speed(speedB);
}

