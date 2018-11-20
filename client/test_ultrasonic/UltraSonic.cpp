#include "UltraSonic.h"

UltraSonic* UltraSonic::singleton = nullptr;

UltraSonic::UltraSonic(int32_t pin, uint32_t max_distance) {
    this->pin = pin;
    this->max_distance = max_distance;

}

bool UltraSonic::init() {
    if (singleton != nullptr) return false;
    singleton = this;

    digitalWrite(pin, LOW);
    pinMode(pin, INPUT);

    pulse_start();

    return true;
}

void UltraSonic::s_pulse_start() {
    singleton->pulse_start();
}

void UltraSonic::s_pulse_end() {
    singleton->pulse_end();
}

void UltraSonic::s_input_start() {
    singleton->input_start();
}

void UltraSonic::s_input_end() {
    singleton->input_end();
}

void UltraSonic::pulse_start() {
    // Turn off the delay before next measurement timer
    timer.end();

    // Set pin to output and start pulse
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);

    // Wait 4 us
    delayMicroseconds(4);

    // End pulse and set pin to input
    digitalWrite(pin, LOW);
    pinMode(pin, INPUT);

    // Wait for reading to go high
    attachInterrupt(pin, s_input_start, RISING);
}

void UltraSonic::pulse_end() {
    // Turn of the pulse timer
    timer.end();

    // Set pin low and set to input
    digitalWrite(pin, LOW);
    pinMode(pin, INPUT);

    // Wait for reading to go high
    attachInterrupt(pin, s_input_start, RISING);
}

void UltraSonic::input_start() {
    // Detach the start of input interrupt
    detachInterrupt(pin);

    // Start the clock
    start_time_us = micros();

    // Attach the end of input interrupt
    attachInterrupt(pin, s_input_end, FALLING);
}

void UltraSonic::input_end() {
    // Detach the end of input interrupt
    detachInterrupt(pin);

    // Record total time
    total_time_us = micros() - start_time_us;

    // Begin the delay before next measurement timer
    timer.begin(s_pulse_start, 200);
}
