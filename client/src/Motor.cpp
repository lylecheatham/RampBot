#include "Motor.hpp"
#include "InterruptDisable.h"

//#define MOTOR_DEBUG_PRINT

std::set<Motor*> Motor::interrupt_list = std::set<Motor*>();
IntervalTimer Motor::intTime = IntervalTimer();


/* Function: Motor()
 * 		constructor - setup the pins and encoder
 * 	Inputs:
 * 		MotorNum   - enum to indicate motor A or B
 * 		PID_enable - whether or not to us PID
 */

Motor::Motor(MotorNum m, bool PID_enable) {
    fix_integration_c__s = 0;
    if (m == MotorA) {
        pwm_pin = M_PWMA;
        in1_pin = M_AIN1;
        in2_pin = M_AIN2;
        enc = std::make_unique<Encoder>(M_AENC1, M_AENC2);
    } else {
        pwm_pin = M_PWMB;
        in1_pin = M_BIN1;
        in2_pin = M_BIN2;
        enc = std::make_unique<Encoder>(M_BENC1, M_BENC2);
    }

    fix_target_speed_c__s = 0;
    fix_integration_c__s = 0;
    saturation = false;

    pwm_val = 0;

    enc->write(0);
    previous_encoder_c = 0;

    fix_previous_error_c__s = 0;
    fix_previous_speed_c__s = 0;

    // Set all pins to outputs
    pinMode(pwm_pin, OUTPUT);
    pinMode(in1_pin, OUTPUT);
    pinMode(in2_pin, OUTPUT);

    // Start assuming PWM is positive
    digitalWrite(in1_pin, 1);
    digitalWrite(in2_pin, 0);
    direction = false;

	pinMode(21, OUTPUT);

    // Disable interrupts before adding this motor to the interrupt list
    InterruptDisable d();
    interrupt_list.insert(this);
}

/* Function: ~Motor()
 * 		standard destructor
 */
Motor::~Motor() {
    InterruptDisable d();
    interrupt_list.erase(interrupt_list.find(this));
}

/* Function: init()
 *      Starts the motor interrupts
 */

bool Motor::init() {
    return intTime.begin(control_interrupt, 1000000 / freq);
}


/* Function: set_speed
 * Inputs:
 *		speed - desired speed [rpm]
 * Outputs:
 * 		None
 */
void Motor::set_speed(float speed_rpm) {
    if (speed_rpm < max_speed_rpm && speed_rpm > -max_speed_rpm) { fix_target_speed_c__s = to_fix_pt(RPM_to_CPS(speed_rpm)); }
}

/* Function: get_speed
 * Inputs:
 * 	 None
 * Outputs:
 * 	 speed [rpm]
 */
float Motor::get_speed() {
    return to_float_pt(CPS_to_RPM(fix_previous_speed_c__s));
}

/* Function: get_count
 * Inputs:
 * 	 None
 * Outputs:
 * 	 Current encoder count [counts]
 */
int32_t Motor::get_count() {
    return enc->read();
}

/* Function: update_pwm
 * 		Updates the pwm pin with the latest value
 * Inputs:
 * 	 None
 * Outputs:
 *	 None
 */
void Motor::update_pwm() {
    if (pwm_val < -max_pwm) {
        pwm_val = -max_pwm;
        saturation = true;
    } else if (pwm_val > max_pwm) {
        pwm_val = max_pwm;
        saturation = true;
    } else {
        saturation = false;
    }

    // Change direction
    // CW
    if (pwm_val > 0 && direction) {
        digitalWrite(in1_pin, 1);
        digitalWrite(in2_pin, 0);

        direction = false;
    }
    // CCW
    else if (pwm_val < 0 && !direction) {
        digitalWrite(in1_pin, 0);
        digitalWrite(in2_pin, 1);

        direction = true;
    }

    // Update the pwm
    analogWrite(pwm_pin, abs(pwm_val));
}


/* Function: PID_control
 *		Executed consistently on a set interval
 * Inputs:
 * 	 None
 * Outputs:
 *	 None
 */
void Motor::PID_control() {
    // get the current encoder count
    int32_t current_encoder_c = get_count();

    // calculate our current speed in fixed point
    int32_t fix_current_speed_c__s = to_fix_pt((current_encoder_c - previous_encoder_c) * freq);

    // store the encoder value for next loop
    previous_encoder_c = current_encoder_c;

    // calculate the error
    int32_t fix_error_c__s = fix_target_speed_c__s - fix_current_speed_c__s;

    // Proportional Value
    int32_t fix_p_out_pwm = fix_p_term * fix_error_c__s / to_fix_pt(1.0);

    // Integral Value
    // Saturate the integral if the PWM is already saturating
    if (!saturation) fix_integration_c__s += fix_error_c__s;
    int32_t fix_i_out_pwm = fix_i_term * fix_integration_c__s / to_fix_pt(1.0) / freq;

    // Derivative Value
    int32_t fix_d_out_pwm = fix_d_term * (fix_error_c__s - fix_previous_error_c__s) / to_fix_pt(1.0) * freq;

    // Calculate PID term
    pwm_val = to_int_pt(fix_i_out_pwm + fix_p_out_pwm + fix_d_out_pwm);

    // Add feed forward term
    pwm_val += to_int_pt(CPS_to_RPM(fix_FF_term) * fix_target_speed_c__s / to_fix_pt(1.0));

    // Store error for next calculation
    fix_previous_error_c__s = fix_error_c__s;

#ifdef MOTOR_DEBUG_PRINT
    char buff[200];
    snprintf(buff,
             200,
             "Target Speed:%+5f, Current Speed:%+5f, Error:%+5f, Integration:%+5f, Direction:%d, Encoder Count:%-5ld, PWM:%+5d",
             to_float_pt(fix_target_speed_c__s),
             to_float_pt(fix_current_speed_c__s),
             to_float_pt(fix_error_c__s),
             to_float_pt(fix_integration_c__s),
             direction,
             get_count(),
             pwm_val);
    Serial.println(buff);
#endif

    update_pwm();
}

/* Function: <static> control_interrupt
 * 		Static function to call the interrupts on each instance of Motor
 * Inputs:
 * 	 None
 * Outputs:
 *	 None
 */
void Motor::control_interrupt() {
	digitalWrite(21, 1);
	
    static bool LED_state = false;
    digitalWrite(13, LED_state);
    LED_state = !LED_state;

    for (Motor* motor : interrupt_list) { motor->PID_control(); }

	digitalWrite(21, 0);
	
}
