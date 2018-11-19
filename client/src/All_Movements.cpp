/* All_Movements.hpp
 * 	MTE 380 Design Project
 * 	Original Author(s): Eric Murphy-Zaremba 
 * 	Creation Date: Nov 17 /2018
 *
 * 	Functionality:
 * 		This implements the derived classes in All_Movements.hpp
 */ 


#include "All_Movements.hpp" 

/********************* Drive Distance ***************************/
/* DriveDistance
 * 	Inputs:
 *		dist_  : desired distance to travel (+ for forwards, - for backwards)
 *		mA_    : pointer to the right motor (A)
 *		mb_    : pointer to the left motor (B)
 *		sonar_ : pointer to the ultrasonic sensor object
 *		speed_ : (optional) desired motor speed for the action
 * 	Outputs:
 * 		none
 */
DriveDistance::DriveDistance(int32_t dist_, Motor* mA_, Motor* mB_, NewPing* sonar_, int32_t speed_) :
		dist(dist_),
		mA(mA_),
		mB(mB_),
		sonar(sonar_),
		speed(speed_) 
{
	timeout = dist_/(speed*RPM_TO_VO)*1000 + TIMEOUT_TOL + millis(); // Get timeout criteria
	speed = dist*speed < 0 ? -speed : speed;                         // Account for direction
	last_status = FAILURE;
}

/* update
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		current execution status (SUCCESS / FAILURE / ONGOING)
 */
Status DriveDistance::update()
{
	if(last_status == FAILURE) // check if not started
	{
		// Save previous speeds
		prev_speedA = 0;
		prev_speedB = 0;

		// Set new speed
		mA->set_speed(speed);
		mB->set_speed(speed);

		// Initial Conditions
		dist += sonar->ping_cm(); // add current distance to our travel offset
	}

	int32_t curr_t = millis();
	int32_t curr_d = sonar->ping_cm();

	// Correct orientation
	correct();

	if(curr_t > timeout)
		last_status = FAILURE;

	else if((speed > 0 && dist - curr_d > TOL) || 
					(speed < 0 && curr_d - dist > TOL))
		last_status = ONGOING;

	else
	{
		last_status = SUCCESS;

		// return to original state
		mA->set_speed(prev_speedA);
		mB->set_speed(prev_speedB);
	}

	return last_status;
}

/* correct
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		none
 * 	Notes:
 * 		A positive error would correspond to a positive angle: turning left
 * 		A negative  ||    ||       ||       || negative  ||  : turning right
 *
 * 		PID output would correspond to a delta: positive delta = need to turn right
 * 			apply half of delta to mB, apply half of -delta to mA
 */
void DriveDistance::correct()
{
	//TODO validate encoder values with ultrasonic (complementary filter) 
	//TODO add correction if not driving straight (PID essentially)
	//angle = 0.98 (angle + yaw_angle*time_interval) + 0.02*enc_angle;

	// calculate error as a combination of encoder and yaw
	


	// basic PID control to compensate for the error
    // Proportional Value
/*
 *    float p_out = kp * error;
 *
 *    // Integral Value=
 *    integration += error;
 *    float i_out = ki * integration * dt;
 *
 *    // Derivative Value
 *    float d_out = kd * (error - previous_error) / dt;
 *
 *    previous_error = error;
 *    float delta = p_out + i_out _ d_out;
 *
 *    // Set new speeds
 *    mB->set_speed(speed + delta/2);
 *    mA->set_speed(speed - delta/2);
 */
}

/********************* Turn Angle ***************************/
/* TurnAngle
 * 	Inputs:
 *		angle_ : desired angle to turn (+ for left, - for right)
 *		mA_    : pointer to the right motor (A)
 *		mb_    : pointer to the left motor (B)
 *		imu_   : pointer to the imu object
 *		speed_ : (optional) desired motor speed for the action
 * 	Outputs:
 * 		none
 */
TurnAngle::TurnAngle(int32_t angle_, Motor* mA_, Motor* mB_, IMU* imu_, int32_t speed_) :
		angle(angle_),
		imu(imu_),
		speed(speed_)
{
	// Set driving and pivot motor: mA is right, mB is left
	if(angle > 0) //right turn
	{
		mDrive = mB_;
		mPivot = mA_;
		right_turn = true;
	}
	else // left turn
	{
		mDrive = mA_;
		mPivot = mB_;
		right_turn = false;
	}
	
	timeout = angle_ < 0 ? TIMEOUT_TOL*(-angle_)/45 : TIMEOUT_TOL*(angle_)/45;
   	timeout += millis(); // get timeout criteria
	speed = speed < 0 ? -speed : speed;   		// only work for forward turns currently
	speed = 0;
	last_status = FAILURE;
}

/* update
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		current execution status (SUCCESS / FAILURE / ONGOING)
 */
Status TurnAngle::update()
{
	if(last_status == FAILURE)
	{
		// Save previous speeds
		prev_speedD = 0;// mDrive->get_speed();
		prev_speedP = 0;//mPivot->get_speed();
		
		// Set turning speed
		mDrive->set_speed(0);
		mPivot->set_speed(0);

		// Initial Conditions
		start_angle = imu->get_yaw(); //TODO make sure account for degrees popping back over
		prev_angle = 0;
		start_enc  = mDrive->get_count();
	}

	curr_t = micros();
	prev_t = curr_t;

	// Complementary filter of encoder and imu values for current angle
	float enc_ang = encoder_angle();
	float imu_ang = imu_angle();

	//prev_angle = 0.98 * (prev_angle + imu_ang*(curr_t - prev_t)) + 0.02*enc_ang;

	prev_angle = imu_ang;

	//if(curr_t - prev_t > 0)
	while(1){
		delayMicroseconds(1);
		prev_angle = imu_angle();
		curr_t = micros();
		pid_update();
		prev_t = curr_t;	
		mDrive->set_speed(speed);
		
	}

	mDrive->set_speed(0);

	Serial.print(" Turn Angle: ");
	Serial.println(prev_angle);
	//Serial.print(" Enc Angle: ");
	//Serial.println(enc_ang);
	/*
	if(curr_t/1000 > timeout)
		last_status = FAILURE;
	else{
		last_status = ONGOING;
	}*/
	// else if(abs(angle-prev_angle) > TOL)
	// 	last_status = ONGOING;
	// else
	// {
	// 	last_status = SUCCESS;

	// 	// Reset original state
	// 	mDrive->set_speed(prev_speedD);
	// 	mPivot->set_speed(prev_speedP);
	// }
	
	//return last_status;
	return SUCCESS;
}

/* encoder_angle
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		current angle calculated based on encoder values
 */

float TurnAngle::encoder_angle()
{
	float	dx = mDrive->get_count() - start_enc; // get encoder count so far
	dx *= (2*PI*D_O_WHEEL/2); // translate to linear distance

	dx *= 1000; //Try to avoid truncation

	dx /= COUNTS_REV;			
	
	float angle = dx/WHEELBASE*RAD_TO_DEG;

	return right_turn ? angle/1000 : -angle/1000; // Reset truncation fix
}

/* imu_angle
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		current angle based on imu
 */
float TurnAngle::imu_angle() 
{
	Serial.println(imu->get_yaw()-start_angle);
	return imu->get_yaw() - start_angle; //TODO account for degrees popping back over
}

/* pid_update
 *	Inputs:
 *		none
 *	Outputs:
 *		none
 */
void TurnAngle::pid_update()
{
	float dt = 1000000/(curr_t - prev_t);

    // Add error
    float error = angle - prev_angle;

    // Proportional Value
    float p_out = k_p * error;

    // Integral Value=
    integration += error;
    float i_out = k_i * integration / dt;

    // Derivative Value
    float d_out = k_d * (error - prev_error) * dt;

	// Get pwm val
    speed = static_cast<int32_t>(p_out + i_out + d_out);
	speed *= 0.5;
    prev_error = error;
	
	// Serial.print(" dt: ");
	// Serial.println(dt);
	// Serial.print(" New Speed: ");
	// Serial.println(speed);
	// Serial.print(" i_out: ");
	// Serial.println(i_out);
	// Serial.print(" p_out: ");
	// Serial.println(p_out);
}

