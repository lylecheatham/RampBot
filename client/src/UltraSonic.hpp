#pragma once

#include <cstdint>
#include <Arduino.h>

class UltraSonic {
public:
    UltraSonic(int32_t pin, uint32_t max_distance);
    bool init();
    uint32_t ping_cm();
    uint32_t total_time_us;

private:
    void pulse_start();
    void pulse_end();
    void input_start();
    void input_end();
    static constexpr uint32_t length = 1000;

    static void s_pulse_start();
    static void s_pulse_end();
    static void s_input_start();
    static void s_input_end();

    static UltraSonic *singleton;
    IntervalTimer timer;
    int32_t pin;
    uint32_t start_time_us;
    uint32_t max_distance;

    //circular buffer
    uint32_t curr_index;
    int32_t dist_array[length];
};
