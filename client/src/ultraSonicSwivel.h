#pragma once

#include "constants.h"

#include <Arduino.h>
#include <NewPing.h>
#include <PWMServo.h>
#include <cstdint>
#include <array>

/* Simple wrapper to take care of 0 values in new ping data*/
class NewPingWrap {
public:
	NewPingWrap(uint8_t trig, uint8_t echo, uint32_t dist = 300) : np(trig,echo,dist) {}
	~NewPingWrap() {};

	uint32_t ping_cm() {
		uint32_t cm = 0, cnt = 0, idx = 0, sum = 0;
		/*
		 *while(cnt < window) {
		 *    cm = np.ping_cm();
		 *    if (cm != 0) {
		 *        cnt++;
		 *        vals[idx] = cm;
		 *        idx++;
		 *        sum += cm;
		 *    }	
		 *}		
		 *return sum/window;
		 */
		cm = np.ping_cm();
		while(abs(cm-prev_val) >10)
		{
			prev_val = cm;
			cm = np.ping_cm();
		}

		prev_val = cm;
		return cm;
	}
	
private: 
	static const uint32_t window = 3;
	NewPing np;
	//std::array<uint32_t, window> vals = {{0}};
	uint32_t prev_val = 400;
};

/* Class to handle controlling the servo for the ultrasonic sensor */
class UltraSonicSwivel {
public:
    UltraSonicSwivel(uint8_t servo_pin, uint8_t us_pin, uint32_t millis_per_deg);
    ~UltraSonicSwivel();

    bool set_position(int32_t new_position);
    int32_t get_position();
    bool get_settled();

    int32_t get_distance();

    typedef struct {
        int32_t distance;
        int32_t angle;
        bool settled;
    } swivel_data;

    swivel_data get_reading();

private:
    PWMServo servo;
    NewPing sensor;

    int servo_set_pos;
    int servo_prev_pos;

    uint32_t millis_per_deg;

    uint32_t servo_set_timestamp;
};
