#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Encoder.h>
#include <IntervalTimer.h>
#include <memory>
#include <set>
#include "constants.h"

#define CPR_S 20.05  // 10.28 //counts_per_rev/60s


#define PWM_CONV 0.65            // converts pid output to pwm val
#define LIMIT 2000               // pwm limit
#define FEED_FWD (27.5 / CPR_S)  // experimentally found the feed forward constant
#define FEED_FWD_REAL 27.5


class Motor {
private:
    static constexpr uint32_t frac_bits = 6;
    static constexpr uint32_t counts__rev = 1203;
    static constexpr uint32_t seconds__minute = 60;

    static constexpr inline float RPM_to_CPS(float RPM) {
        return RPM * counts__rev / seconds__minute;
    }

    static constexpr inline float CPS_to_RPM(float CPS) {
        return CPS / counts__rev * seconds__minute;
    }

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

public:
    Motor(MotorNum m, bool PID_enable = true);  // Constructor requires motor name and PID enable
    ~Motor();                                   // Basic destructor

    void set_speed(float speed);  // Set the targt speed
    float get_speed();            // Get the last calculated speed
    int32_t get_count();            // Read the encoder value

    static void control_interrupt();  // Static function to handle the control

private:
    void PID_control();  // PID control loop
    void update_pwm();   // Update the value of the PWM pin


public:
    static IntervalTimer intTime;  // Timer to handle PID updating
    static int32_t freq;           // Frequency of updates


    int32_t fix_target_speed_c__s;

    static int32_t fix_d_term;
    static int32_t fix_k_term;
    static int32_t fix_i_term;
    int32_t fix_integration_c__s;

    int32_t pwm_val;  // Value to write to the PWM pin

    int32_t previous_encoder_value;  // Last read encoder value
    int32_t fix_previous_speed_c__s;
    int32_t fix_previous_error_c__s;


private:
    static constexpr float max_speed = 600;    // [rpm]
    std::unique_ptr<Encoder> enc;            // Encoder for the motor
    int32_t pwm_pin, in1_pin, in2_pin;       // The relevant pins
    int8_t direction = 0;                    // zero for CW, one for CCW
    static std::set<Motor*> interrupt_list;  // Pointers to all constructed motors to iterate over for PID
};

#endif
