#include "Motor.hpp"
#include "InterruptDisable.h"


std::set<Motor*> Motor::interrupt_list = std::set<Motor*>();
int32_t Motor::freq = 40;
IntervalTimer Motor::intTime = IntervalTimer();

float Motor::k_term = 0.122;  // 0.12215 // was 0.09 for old motors
float Motor::d_term = 0.001;  // 0.001;
float Motor::i_term = 2.774;  // 10;
int32_t Motor::i_max = 10;

/* Function: Motor()
 * 		constructor - setup the pins and encoder
 * 	Inputs:
 * 		MotorNum   - enum to indicate motor A or B
 * 		PID_enable - whether or not to us PID
 */

Motor::Motor(MotorNum m, bool PID_enable) {
    integration = 0;

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
    previous_speed = 0;

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
void Motor::set_speed(int32_t speed) {
    if (speed < max_speed && speed > -max_speed) {
        target_speed = speed * CPR_S;  // translate to counts per second
    }
}

/* Function: get_speed
 * Inputs:
 * 	 None
 * Outputs:
 * 	 speed [rpm]
 */
int32_t Motor::get_speed() {
    return (int32_t)(previous_speed / CPR_S);
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
    float current_speed = (get_count() - previous_encoder_value) * freq;
    previous_encoder_value = get_count();

    // Add error
    float error = target_speed - current_speed;

    // Proportional Value
    float p_out = k_term * error;

    // Integral Value=
    integration += error;
    float i_out = i_term * integration / freq;

    // Derivative Value
    float d_out = d_term * (error - previous_error) * freq;

    // Get pwm val
    pwm_val = p_out + i_out + d_out;
    pwm_val *= PWM_CONV;

	pwm_val = pwm_val + FEED_FWD*target_speed;

    previous_speed = current_speed;  // TODO: delete maybe?
    previous_error = error;

    // Uncomment for debug info
    // char buff [200];
    // snprintf(buff, 200, "Target Speed: %f   Current Speed: %f   Error:  %f  Integration:  %f   Direction: %d  Encoder Count: %ld", target_speed,
    // current_speed, error, integration, direction, get_count()); Serial.println(buff);


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
