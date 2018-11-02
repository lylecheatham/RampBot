#include "Motor.hpp"
#include "InterruptDisable.h"

/* Function: Motor()
 * 		constructor - setup the pins and encoder 
 * 	Inputs:
 * 		MotorNum   - enum to indicate motor A or B
 * 		PID_enable - whether or not to us PID
 */
Motor::Motor(MotorNum m, bool PID_enable)
{
	if(m == MotorA)
	{
		pwm_pin = M_PWMA;
		in1_pin = M_AIN1;
		in2_pin = M_AIN2;
		enc = std::make_unique<Encoder>(M_AENC1, M_AENC2);
	}
	else
	{
		pwm_pin = M_PWMB;
		in1_pin = M_AIN1;
		in2_pin = M_AIN2;
		enc = std::make_unique<Encoder>(M_BENC1, M_BENC2);
	}

	enc->write(0);
    previous_encoder_value = enc->read();

	// Begin the interval timer
	//intTime.begin(PID_control, 5000);
    {
        InterruptDisable d();
        interrupt_list.push_back(this);
    }

}

/* Function: ~Motor()
 * 		standard destructor 
 */
Motor::~Motor()
{
    InterruptDisable d();
    interrupt_list.remove(this);
}

/* Function: set_speed
 * Inputs:
 *		speed - desired speed [rpm]
 * Outputs:
 * 		None
 */
void Motor::set_speed(int32_t speed)
{
	if(speed < max_speed && speed > -max_speed)
	{
		target_speed = speed*CPR_S; // translate to counts per second
	}

}

/* Function: get_speed
 * Inputs:
 * 	 None
 * Outputs:
 * 	 speed [rpm]
 */
int32_t Motor::get_speed()
{
	return (int32_t)(previous_speed/CPR_S);
}

/* Function: get_count
 * Inputs:
 * 	 None
 * Outputs:
 * 	 Current encoder count [counts]
 */
int32_t Motor::get_count()
{
	return enc->read();
}

/* Function: PID_control
 *		Executed consistently on a set interval
 * Inputs:
 * 	 None
 * Outputs:
 *	 None 	 
 */
void Motor::PID_control() {

	// Add error
	double error = target_speed - previous_speed;

	//Proporsional Value
	double p_out = k_term*error;

	//Integral Value
	double i_out = i_term*error/freq;

	//Derivative Value
	double d_out = d_term*(get_count() - previous_speed)*freq;

	pwm_val = p_out + i_out + d_out;

	previous_speed = get_count();
}

void Motor::control_interrupt(){
    for (Motor* motor : interrupt_list){
        motor->PID_control();
    }
}
