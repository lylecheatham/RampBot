#pragma once

#include <cstdint>

class InterruptDisable {
public:
    InterruptDisable();
    ~InterruptDisable();

private:
    static int32_t counter;
    static bool interrupts_enabled();
    static int32_t interrupt_disable_count();
};
