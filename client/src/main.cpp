/* main.cpp
 * MTE 380 Design Project
 * Original Author(s): Lyle Cheatham / Eric Murphy-Zaremba / Conner Currie
 * Creation Date: Oct 20 /2018
 */


//#include <ADC.h>
#include <Arduino.h>
#include <NewPing.h>
#include <PWMServo.h>

#include "DemoInterface.hpp"
#include "constants.h"
#include "packet.h"
#include "sensor_packet.h"
//#include "state_machine.hpp"
#include "IMU.hpp"

// Hacky stuff, don't remove
extern "C" {
int _getpid() {
    return -1;
}
int _kill(int pid, int sig) {
    return -1;
}
int _write() {
    return -1;
}
}


void setup() {
    /* Await serial connection */
    // TWBR = 12;
    Serial.begin(38400);  // 400 kbit/sec I2C speed
    while (!Serial) {
    };

    pinMode(STD_LED, OUTPUT);
    digitalWrite(STD_LED, HIGH);

    /* Setup state machine */
    // state_machine machine;
    IMU myIMU;

    while (1) {
        myIMU.print_values();
    }
}

void loop() {}
