#include "InterruptDisable.h"
#include <Arduino.h>


InterruptDisable::InterruptDisable(){
    noInterrupts();
    counter++;
}

InterruptDisable::~InterruptDisable(){
    interrupts();
    counter--;
}

bool InterruptDisable::interrupts_enabled(){
    return counter == 0;
}

int32_t InterruptDisable::interrupt_disable_count(){
    return counter;
}

int32_t InterruptDisable::counter = 0;
