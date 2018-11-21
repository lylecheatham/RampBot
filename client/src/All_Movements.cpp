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
DriveDistance::DriveDistance(int32_t dist_, Motor* mA_, Motor* mB_, UltraSonicSwivel* servo_, IMU* imu_, int32_t speed_) :
		dist(dist_),
		mA(mA_),
		mB(mB_),
		servo(servo_),
		imu(imu_)
{
	k_p = 1.5;
	k_i = 0.001;
	k_d = 0.001;
	freq = 1000;
	integration = 0;

	timeout = 1000*dist_/(speed_*RPM_TO_VO) + TIMEOUT_TOL + millis(); // Get timeout criteria
	base_speed = dist*speed_ < 0 ? -speed_ : speed_;                  // Account for direction
	speedA = 0;
	speedB = 0;
	last_status = INIT;
}

/* update
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		current execution status (SUCCESS / FAILURE / ONGOING)
 */
Status DriveDistance::run()
{
	// Ensure motors stopped to begin with
	mA->set_speed(speedA);
	mB->set_speed(speedB);

	// Initial Conditions
	encA_start  = mA->get_count();
	encB_start  = mB->get_count();
	dist = get_dist() - dist;
	dist = dist < 5 ? 5 : dist;	    //Ensure no negative distances	
	target_angle = imu->get_yaw();
	

	// Using combination of speed and error to represent stabilizing on the correct point
	int32_t curr_dist = get_dist();
	timeout = millis() + 10000;

	// Speed adjustment
	int32_t speed_adj = 0;
	uint32_t curr_t = 0;

	// Loop until timed out
	while(curr_t < timeout){ 
		curr_angle = imu->get_yaw();
		while(millis()-curr_t < 1); // only update at 1KHz
		speed_adj = pid_control(); 		
		curr_t = millis();
		speedA = base_speed + speed_adj;
		speedB = base_speed - speed_adj;

		mA->set_speed(speedA);
		mB->set_speed(speedB);

		curr_dist = get_dist();
		Serial.println(curr_dist);
		// Success condition
		/*
		if(speedA > 0 && speedA*speedB > 0 && curr_dist < dist)
			break;
		else if(speedA < 0 && speedA*speedB > 0 && curr_dist > dist)
			break;
		*/
	}

	mA->set_speed(0); // ensure motor stopped at this point
	mB->set_speed(0);

	return curr_t > timeout ? TIMEOUT : SUCCESS;
}

/* encoder_delta
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		Difference between left and right encoders
 */
float DriveDistance::encoder_delta()
{
	return mB->get_count() -encB_start - mA->get_count() + encA_start;
}

/* encoder_dist_cm
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		Distance travelled so far as measured by encoders
 */
float DriveDistance::encoder_dist_cm()
{
/*
 *    float ret_dist_a = mA->get_count() - encA_start;
 *    ret_dist_a *= (2*PI*D_O_WHEEL/2); // translate to linear distance
 *    ret_dist_a /= COUNTS_REV*10;	
 *
 *    float ret_dist_b = mB->get_count() - encB_start;
 *    ret_dist_b *= (2*PI*D_O_WHEEL/2); // translate to linear distance
 *    ret_dist_b /= COUNTS_REV*10;	
 */

	return (mA->get_count() - encA_start + mB->get_count() - encB_start) * (2*PI*D_O_WHEEL) 
			/ (2*COUNTS_REV*10) /2;
}

/* get_dist
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		Gets the currently read distance
 */
int32_t DriveDistance::get_dist()
{
	return static_cast<int32_t>(encoder_dist_cm());
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
TurnAngle::TurnAngle(int32_t angle_, Motor* mA_, Motor* mB_, IMU* imu_) :
		imu(imu_)
{
	k_p = 0.75;
	k_i = 1.5;
	k_d = 0.01;
	freq = 1000;
	integration = 0;
	target_angle = angle_;

	// Set driving and pivot motor: mA is right, mB is left
	if(target_angle > 0) //right turn
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
	base_speed = 0;
	last_status = INIT;
}

/* update
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		current execution status (SUCCESS / FAILURE / ONGOING)
 */
Status TurnAngle::run()
{	
	// Ensure motors stopped
	mDrive->set_speed(0);
	mPivot->set_speed(0);

	// Initial Conditions
	start_enc  = mDrive->get_count();
	start_angle = imu->get_yaw();

	// Using combination of speed and error to represent stabilizing on the correct point
	uint32_t curr_t = 0;
	int32_t speed_adj = 0;
	error = 10*TOL;
	while(abs(error) > TOL && curr_t < timeout){ 
		curr_angle = imu_angle();
		while(millis()-curr_t < 1); // only update at 1KHz
		speed_adj = pid_control(); 
		curr_t = millis();
		base_speed = right_turn ? -speed_adj : speed_adj;
		mDrive->set_speed(base_speed);
	}

	mDrive->set_speed(0); // ensure motor stopped at this point

	return curr_t > timeout ? TIMEOUT : SUCCESS;
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

	dx /= COUNTS_REV;			
	
	float angle = dx/WHEELBASE*RAD_TO_DEG;
	angle *= 1.4; // account for ~30% undershoot in the encoder angle for some reason

	return right_turn ? angle : -angle; 
}

/* imu_angle
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		current angle based on imu
 */
float TurnAngle::imu_angle() 
{
	return imu->get_yaw() - start_angle; 
}

/********************* Ramp Movement ***************************/
/* Ramp Movement
 * 	Inputs:
 *		mA_    : pointer to the right motor (A)
 *		mb_    : pointer to the left motor (B)
 *		imu_   : pointer to the imu object
 * 	Outputs:
 * 		none
 */
RampMovement::RampMovement(Motor* mA_, Motor* mB_, IMU* imu_) :
		imu(imu_),
		mA(mA_),
		mB(mB_)
{
	
}

Status RampMovement::run(){
	
}