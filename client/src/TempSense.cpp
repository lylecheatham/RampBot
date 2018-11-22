#include "TempSense.h"
#include <ADC.h>
#include <cmath>

extern ADC adc;

TempSense::TempSense(int pin, bool continuous) {
    this->pin = pin;
    adc.setResolution(12, 0);
    adc.setResolution(12, 1);

    // Try to set up continuous read
    this->continuous = false;
    if (continuous) { this->continuous = adc.startContinuous(pin); }

    // Set to defaults
    this->B = default_B;
    this->R_25C = default_R_25C;
    this->R_bal = default_R_bal;
    this->config = default_config;
    this->r_inf = this->R_25C * exp((-this->B / 25.0));
}

TempSense::~TempSense() {
    adc.stopContinuous(pin);
}

float TempSense::get_temp() {
    // Get the ADC value properly for continuous and non cont
    int32_t adc_val;
    if (continuous) {
        adc_val = adc.analogReadContinuous(pin);
    } else {
        adc_val = adc.analogRead(pin);
    }

    if (config == low_side) { adc_val = 4095 - adc_val; }

    float voltage = adc_val / 4095.0;

    float resistance = R_bal / voltage - R_bal;

    float temp = B / log(resistance / r_inf);

    return temp;
}
