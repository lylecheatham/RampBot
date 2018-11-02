#pragma once

#include "constants.h"

#include <Arduino.h>
#include <cstdint>
#include <NewPing.h>
#include <PWMServo.h>


class UltraSonicSwivel
{
public:
    UltraSonicSwivel(uint8_t servo_pin, uint8_t us_pin, uint32_t millis_per_deg);
    ~UltraSonicSwivel();

    int32_t set_position();
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
