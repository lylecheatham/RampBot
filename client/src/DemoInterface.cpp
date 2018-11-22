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

int32_t DemoInterface::speedA = 0;
int32_t DemoInterface::speedB = 0;

std::string DemoInterface::error_string = "";

// Initialize swivel Variable
UltraSonicSwivel* DemoInterface::servo = nullptr;

DemoInterface::DemoInterface() {
    mA = new Motor(MotorA, true);
    mB = new Motor(MotorB, true);
    servo = new UltraSonicSwivel(S_PULSE, U_PING, 1);

    pinMode(M_STDBY, OUTPUT);
    if (!Motor::init()) { error_string.append("interrupt init fail;"); }
    stop();
}

DemoInterface::~DemoInterface() {
    delete mA;
    delete mB;
    delete servo;
}

bool DemoInterface::run_command(int8_t key) {
    if (key == 10 || key == 13) {
        standby();
        return true;
    }

    if (key > 'z' || key < 'a') return false;

    (*commands[control_keys[key - 'a']])();

    return true;
}

void DemoInterface::standby() {
    digitalWrite(M_STDBY, 1);
}

void DemoInterface::update_speeds() {
    if (speedA > 150) speedA = 150;
    if (speedA < -150) speedA = -150;
    if (speedB > 150) speedB = 150;
    if (speedB < -150) speedB = -150;
    mA->set_speed(speedA);
    mB->set_speed(speedB);
}

void DemoInterface::error() {
#ifdef DEBUG_PRINT
    Serial.println("Wrong key");
#endif
}

void DemoInterface::move_forward() {
#ifdef DEBUG_PRINT
    Serial.println("Moving forward");
#endif
    if (abs(speedA) < abs(speedB))
        speedB = speedA;
    else
        speedA = speedB;

    speedA += 5;
    // speedB -= 5;
    speedB += 5;
    update_speeds();
}

void DemoInterface::move_backward() {
#ifdef DEBUG_PRINT
    Serial.println("Moving backward");
#endif
    if (abs(speedA) < abs(speedB))
        speedB = speedA;
    else
        speedA = speedB;


    speedA -= 5;
    // speedB += 5;
    speedB -= 5;
    update_speeds();
}

void DemoInterface::stop() {
#ifdef DEBUG_PRINT
    Serial.println("Stopping");
#endif

    // Put both motors in STOP
    speedA = 0;
    speedB = 0;
    update_speeds();
}

void DemoInterface::turn_left() {
#ifdef DEBUG_PRINT
    Serial.println("Turning Left");
#endif

    speedA += 5;
    speedB = 0;
    update_speeds();
}

void DemoInterface::turn_right() {
#ifdef DEBUG_PRINT
    Serial.println("Turning right");
#endif

    speedA = 0;
    speedB += 5;
    update_speeds();
}

void DemoInterface::servo_left() {
    int32_t current_pos = servo->get_position();
    servo->set_position(current_pos - 1);
}

void DemoInterface::servo_right() {
    int32_t current_pos = servo->get_position();
    servo->set_position(current_pos + 1);
}

void DemoInterface::print_error() {
    Serial.println(error_string.c_str());
}


int8_t DemoInterface::get_char() {
    if (Serial.available() > 0) return Serial.read();
    return -1;
}
