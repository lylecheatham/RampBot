/* main.cpp
 * MTE 380 Design Project
 * Original Author(s): Lyle Cheatham / Eric Murphy-Zaremba / Conner Currie
 * Creation Date: Oct 20 /2018
 */


#include <ADC.h>
#include <Arduino.h>
#include <MPU9250.h>
#include <NewPing.h>
#include <PWMServo.h>

#include "DemoInterface.hpp"
#include "constants.h"
#include "packet.h"
#include "sensor_packet.h"
#include "state_machine.hpp"


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

DemoInterface* demo;

ADC adc();

void setup() {
    analogWriteResolution(12);

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    
    // Startup the State Machine
    state_machine machine;
    //Serial.println("Established State Machine");
    //Create ultrasonic instance
    //NewPing sonar(U_PING, U_PING, 300);

    // Loop goes here
    while (1){
    }
}

void loop() {}
