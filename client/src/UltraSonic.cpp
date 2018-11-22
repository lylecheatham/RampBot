#include "UltraSonic.hpp"
#include "constants.h"

//#define INT_SIGS

UltraSonic* UltraSonic::singleton = nullptr;

UltraSonic::UltraSonic(int32_t pin, uint32_t max_distance) {
    this->pin = pin;
    this->max_distance = max_distance;
    this->valid_reading = false;
}

bool UltraSonic::init() {
    if (singleton != nullptr) return false;
    singleton = this;

    curr_index = length - 1;

    digitalWrite(pin, LOW);
    pinMode(pin, INPUT);

    pulse_start();
    delay(100);
	
	pinMode(GRN_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
    pinMode(DBG_1, OUTPUT);

    if (!valid_readings()) 
	{
#ifndef INT_SIGS
		digitalWrite(RED_LED, 1);
#endif
		Serial.println("Ultrasonic Failed init");
		return false;
	}
	
    Serial.println("Ultrasonic Initialized");
    return true;
}

void UltraSonic::s_pulse_start() {
#ifdef INT_SIGS
    digitalWrite(GRN_LED, 1);
#endif
    singleton->pulse_start();
#ifdef INT_SIGS
    digitalWrite(GRN_LED, 0);
#endif
}

void UltraSonic::s_input_start() {
#ifdef INT_SIGS
    digitalWrite(RED_LED, 1);
#endif
    singleton->input_start();
#ifdef INT_SIGS
    digitalWrite(RED_LED, 0);
#endif
}

void UltraSonic::s_input_end() {
#ifdef INT_SIGS
    digitalWrite(DBG_1, 1);
#endif
    singleton->input_end();
#ifdef INT_SIGS
    digitalWrite(DBG_1, 0);
#endif
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
    attachInterrupt(pin, s_input_start, HIGH);

    // Also set a timer for 100 ms from now in case we get lost
    timer.begin(s_pulse_start, 100000);
}

void UltraSonic::input_start() {
    // Detach the start of input interrupt
    detachInterrupt(pin);

    // Start the clock
    start_time_us = micros();

    // Attach the end of input interrupt
    attachInterrupt(pin, s_input_end, LOW);
}

void UltraSonic::input_end() {
    // Detach the end of input interrupt
    detachInterrupt(pin);

    // Record total time
    total_time_us = micros() - start_time_us;
    if (total_time_us != 0) {
        dist_array[curr_index] = total_time_us;
        if (curr_index == 0)
            curr_index = length - 1;
        else
            curr_index -= 1;
    }

    valid_reading = true;

    // End the "in case we get lost" timer
    timer.end();

    // Begin the delay before next measurement timer
    timer.begin(s_pulse_start, 10000);
}

uint32_t UltraSonic::ping_cm() {
    int32_t threshold = 10;
    bool success = false;
    uint32_t avg = 0;
    uint32_t count = 0;
    uint32_t time_us = 0;

    int32_t index = curr_index;

    // Fancy Robust Code Do Not Touch

    // if(dist_array[index] <= 0){
    //     time_us -= dist_array[index];
    // } else {
    //     time_us += 966 + dist_array[index];
    //     if(abs(dist_array[index] - dist_array[(index+1)%length]) < threshold){
    //         count++;
    //         avg += dist_array[index];
    //     }
    // }
    // index = (index + 1) % length;

    // while(time_us < 100000){
    //     if(dist_array[index] <= 0){
    //         time_us -= dist_array[index] - 966;
    //     } else {
    //         time_us += dist_array[index] + 966;
    //         if(abs(dist_array[index] - dist_array[(index+1)%length]) < threshold || abs(dist_array[index] - dist_array[(index-1)%length]) < threshold){
    //             count++;
    //             avg += dist_array[index];
    //         }
    //     }
    //     index = (index + 1) % length;
    // }

    // avg /= count;
    // avg /= 29*2;
    while (!success) {
        if (abs(dist_array[index] - dist_array[(index + 1) % length]) < threshold || abs(dist_array[index] - dist_array[(index + 2) % length]) < threshold) {
            success = true;
        } else {
            index = (index + 1) % length;
        }
    }

    return dist_array[index] / (29 * 2);
}

bool UltraSonic::valid_readings() {
    return valid_reading;
}
