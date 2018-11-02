#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "constants.h"
#include <Encoder.h>
#include <IntervalTimer.h>
#include <set>
#include <memory>

#define CPR_S 10.28 //counts_per_rev*60s
#define PWM_CONV 1  //converts pid output to pwm val

class Motor
{
  public:
	// Static timer to handle the control
	static IntervalTimer intTime;
	static int32_t freq;

  private:
	static const int16_t max_speed = 600; //[rpm]
	std::unique_ptr<Encoder> enc;
	int32_t pwm_pin, in1_pin, in2_pin;
	int8_t  direction = 0; // zero for CW, one for CCW
	static std::set<Motor*> interrupt_list;	

  public:
    Motor(MotorNum m, bool PID_enable = true);
    ~Motor();

    void    set_speed(int32_t speed);
    int32_t get_speed();
    int32_t get_count();

	// Static function to handle the control
    static void control_interrupt();


    float target_speed;

    int32_t i_counter;
    int32_t i_max;

    float d_term = 1;
    float k_term = 1;
    float i_term = 1;
	float integration = 0;

    int32_t pwm_val;

    int32_t previous_encoder_value;
    float	previous_speed;
	float   previous_error;

  private:
    void PID_control();		
	void update_pwm();

};

#endif
