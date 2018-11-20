#include "Motor.hpp"
#include "InterruptDisable.h"


std::set<Motor*> Motor::interrupt_list = std::set<Motor*>();
int32_t Motor::freq = 40;
IntervalTimer Motor::intTime = IntervalTimer();

int32_t Motor::fix_k_term = to_fix_pt(0.122);
int32_t Motor::fix_d_term = to_fix_pt(0.001);
int32_t Motor::fix_i_term = to_fix_pt(2.774);

/* Function: Motor()
 * 		constructor - setup the pins and encoder
 * 	Inputs:
 * 		MotorNum   - enum to indicate motor A or B
 * 		PID_enable - whether or not to us PID
 */

Motor::Motor(MotorNum m, bool PID_enable) {

    fix_integration_c__s = 0;
    pwm_val = 0;

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

    enc->write(0);
    previous_encoder_value = 0;

    fix_previous_speed_c__s = 0;
    fix_previous_error_c__s = 0;

    fix_previous_speed_c__s = 0;
    pinMode(pwm_pin, OUTPUT);
    pinMode(in1_pin, OUTPUT);
    pinMode(in2_pin, OUTPUT);

    digitalWrite(in1_pin, 1);
    digitalWrite(in2_pin, 0);


    {
        InterruptDisable d();
        interrupt_list.insert(this);
    }
}

/* Function: ~Motor()
 * 		standard destructor
 */
Motor::~Motor() {
    InterruptDisable d();
    interrupt_list.erase(interrupt_list.find(this));
}

/* Function: set_speed
 * Inputs:
 *		speed - desired speed [rpm]
 * Outputs:
 * 		None
 */
void Motor::set_speed(float speed) {
    if (speed < max_speed && speed > -max_speed) {
        fix_target_speed_c__s = to_fix_pt(RPM_to_CPS(speed));
    }
}

/* Function: get_speed
 * Inputs:
 * 	 None
 * Outputs:
 * 	 speed [rpm]
 */
float Motor::get_speed() {
    return CPS_to_RPM(to_float_pt(fix_previous_speed_c__s));
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
    if (pwm_val < -LIMIT)
        pwm_val = -LIMIT;
    else if (pwm_val > LIMIT)
        pwm_val = LIMIT;

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
    int32_t current_encoder_count = get_count();

    // calculate our current speed in fixed point
    int32_t fix_current_speed_c__s = to_fix_pt((current_encoder_count - previous_encoder_value) * freq);

    // store the encoder value for next loop
    previous_encoder_value = current_encoder_count;

    // calculate the error
    int32_t fix_error_c__s = fix_target_speed_c__s - fix_current_speed_c__s;

    // Proportional Value
    int32_t fix_p_out_pwm = fix_k_term * fix_error_c__s / to_fix_pt(1.0);

    // Integral Value=
    fix_integration_c__s += fix_error_c__s;
    int32_t fix_i_out_pwm = fix_i_term * fix_integration_c__s / to_fix_pt(1.0) / freq;

    // Derivative Value
    int32_t fix_d_out_pwm = fix_d_term * (fix_error_c__s - fix_previous_error_c__s) / to_fix_pt(1.0) * freq;

    // Get pwm val
    pwm_val = to_int_pt(fix_i_out_pwm + fix_p_out_pwm + fix_d_out_pwm);
    pwm_val *= PWM_CONV;

	pwm_val = pwm_val + to_int_pt(to_fix_pt(CPS_to_RPM(FEED_FWD_REAL)) * fix_target_speed_c__s / to_fix_pt(1.0));

    fix_previous_error_c__s = fix_error_c__s;

    char buff [200];
    snprintf(buff, 200, "Target Speed:%+5f, Current Speed:%+5f, Error:%+5f, Integration:%+5f, Direction:%d, Encoder Count:%-5ld, PWM:%+5d",
             to_float_pt(fix_target_speed_c__s),
             to_float_pt(fix_current_speed_c__s),
             to_float_pt(fix_error_c__s),
             to_float_pt(fix_integration_c__s),
             direction,
             get_count(),
             pwm_val
             );
    Serial.println(buff);


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
    static bool LED_state = false;
    digitalWrite(13, LED_state);
    LED_state = !LED_state;

    for (Motor* motor : interrupt_list) {
        motor->PID_control();
    }
}
