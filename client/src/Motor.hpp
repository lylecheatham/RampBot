#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Encoder.h>
#include <IntervalTimer.h>
#include <memory>
#include <set>
#include "constants.h"



#define PWM_CONV 0.65            // converts pid output to pwm val

// FIXED POINT FUNCTIONS
static constexpr uint32_t frac_bits = 6;  // 26.6 signed fixed point numbers

static constexpr inline int32_t to_fix_pt(float float_pt) {
    return (int32_t)(float_pt * ((int32_t)1 << frac_bits));
}

static constexpr inline int32_t to_fix_pt(int32_t int_pt) {
    return (int32_t)(int_pt * ((int32_t)1 << frac_bits));
}

static constexpr inline float to_float_pt(int32_t fix_pt) {
    return ((float)fix_pt) / ((int32_t)1 << frac_bits);
}

static constexpr inline int32_t to_int_pt(int32_t fix_pt) {
    return (fix_pt / ((int32_t)1 << frac_bits));
}

class Motor {
private:
    // LIMITS
    static constexpr float max_speed_rpm = 600;
    static constexpr int32_t max_pwm = 2000;

    // TUNINGS
    static constexpr int32_t freq = 40;  // Frequency of updates

    static constexpr float p_term = 0.122;  // P term
    static constexpr float i_term = 2.774;  // I term
    static constexpr float d_term = 0.001;  // D term
    static constexpr int32_t fix_p_term = to_fix_pt(0.122);  // P term
    static constexpr int32_t fix_i_term = to_fix_pt(2.774);  // I term
    static constexpr int32_t fix_d_term = to_fix_pt(0.001);  // D term
    static constexpr int32_t fix_FF_term = to_fix_pt(27.5);  // Feed Forward term


    // CONSTANTS AND TRANSLATIONS
    static constexpr uint32_t counts__rev = COUNTS_REV;
    static constexpr uint32_t seconds__minute = 60;

    static constexpr inline float RPM_to_CPS(float RPM) {
        return RPM * counts__rev / seconds__minute;
    }

    static constexpr inline int32_t RPM_to_CPS(int32_t RPM) {
        return RPM * counts__rev / seconds__minute;
    }

    static constexpr inline float CPS_to_RPM(float CPS) {
        return CPS / counts__rev * seconds__minute;
    }

    static constexpr inline int32_t CPS_to_RPM(int32_t CPS) {
        return CPS / counts__rev * seconds__minute;
    }


public:
    Motor(MotorNum m, bool PID_enable = true);  // Constructor requires motor name and PID enable
    ~Motor();                                   // Destructor

    static bool init();               // Initialize the control interrupt timer
    void set_speed(float speed_rpm);  // Set the targt speed
    float get_speed();                // Get the last calculated speed
    int32_t get_count();              // Read the encoder value

private:
    void PID_control();               // PID control loop
    void update_pwm();                // Update the value of the PWM pin
    static void control_interrupt();  // Static function to handle the control


    int32_t fix_target_speed_c__s;  // Fixed point target speed in counts/second
    int32_t fix_integration_c__s;   // Fixed point integration in counts/second
    bool saturation;

    int32_t pwm_pin, in1_pin, in2_pin;  // The relevant pin numbers
    int32_t pwm_val;                    // Value to write to the PWM pin
    bool direction;                     // zero for CW, one for CCW

    int32_t previous_encoder_c;       // Last encoder reading in counts
    int32_t fix_previous_speed_c__s;  // Fixed point previous speed in counts/second
    int32_t fix_previous_error_c__s;  // Fixed point previous error in counts/second

    static IntervalTimer intTime;            // Timer to handle PID updating
    static std::set<Motor*> interrupt_list;  // Pointers to all constructed motors to iterate over for PID
    std::unique_ptr<Encoder> enc;            // Encoder for the motor
};

#endif
