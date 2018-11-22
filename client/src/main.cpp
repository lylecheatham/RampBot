/* main.cpp
 * MTE 380 Design Project
 * Original Author(s): Lyle Cheatham / Eric Murphy-Zaremba / Conner Currie
 * Creation Date: Oct 20 /2018
 */


//#include <ADC.h>
#include <Arduino.h>
#include <NewPing.h>
#include <PWMServo.h>
//#include <kinesis.h>

#include "DemoInterface.hpp"
#include "IMU.hpp"
#include "constants.h"
#include "packet.h"
#include "sensor_packet.h"
#include "state_machine.hpp"
#include "ultraSonicSwivel.h"

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
    NVIC_SET_PRIORITY(IRQ_PORTB, 240);
    NVIC_SET_PRIORITY(IRQ_PORTC, 32);
    NVIC_SET_PRIORITY(IRQ_PORTD, 32);

    pinMode(STD_LED, OUTPUT);
    digitalWrite(STD_LED, HIGH);

    while (!(Serial || digitalRead(20))) {};

    /* Setup state machine */
    state_machine machine;

    machine.start();
}

void loop() {}
