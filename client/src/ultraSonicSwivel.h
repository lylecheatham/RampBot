#pragma once

#include <Arduino.h>
#include "constants.h"


class UltraSonicSwivel
{
public:
    UltraSonicSwivel(uint8_t servo_pin, uint8_t us_pin);
    ~UltraSonicSwivel() {} ;

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
    int servo_set_pos;
    uint32_t servo_set_timestamp;
};
