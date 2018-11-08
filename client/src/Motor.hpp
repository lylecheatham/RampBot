#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "constants.h"
#include <Encoder.h>
#include <IntervalTimer.h>
#include <set>
#include <memory>

#define CPR_S 20.05 //10.28 //counts_per_rev*60s
#define PWM_CONV 0.65  //converts pid output to pwm val
#define LIMIT 2000 //pwm limit


class Motor
{
  public:
    Motor(MotorNum m, bool PID_enable = true); 	// Constructor requires motor name and PID enable
    ~Motor();								   	// Basic destructor

    void    set_speed(int32_t speed);			// Set the targt speed
    int32_t get_speed();						// Get the last calculated speed
    int32_t get_count();						// Read the encoder value

	static void control_interrupt();			// Static function to handle the control

  private:
    void PID_control();							// PID control loop
	void update_pwm();							// Update the value of the PWM pin



  public:
	static IntervalTimer intTime;			// Timer to handle PID updating
	static int32_t freq;					// Frequency of updates

	
    float target_speed;						// Desired motor speed (counts/sec)

    int32_t i_counter;						//
    static int32_t i_max;							//

    static float d_term;				// Kd
    static float k_term;				// Kp
    static float i_term;				// Ki
	float integration = 0;					// Continuous value of ongoing integration
	
    int32_t pwm_val;						// Value to write to the PWM pin

    int32_t previous_encoder_value;			// Last read encoder value
    float	previous_speed;					// Last calculated speed
	float   previous_error;					// Last calculated error


  private:
	static const int16_t max_speed = 600;	// [rpm]
	std::unique_ptr<Encoder> enc;			// Encoder for the motor
	int32_t pwm_pin, in1_pin, in2_pin;		// The relevant pins
	int8_t  direction = 0; 					// zero for CW, one for CCW
	static std::set<Motor*> interrupt_list;	// Pointers to all constructed motors to iterate over for PID

};

#endif
