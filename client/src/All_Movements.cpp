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
		mA->set_speed(speed);
		mB->set_speed(speed);
		dist += sonar->ping_cm(); // add current distance to our travel offset
	}

	int32_t curr_t = millis();
	int32_t curr_d = sonar->ping_cm();

	//TODO validate encoder values with ultrasonic (complementary filter)
	//TODO add correction if not driving straight
	//angle = 0.98 (angle + yaw_angle*time_interval) + 0.02*enc_angle;

	if(curr_t > timeout)
		last_status = FAILURE;
	else if((speed > 0 && dist - curr_d > TOL) || 
					(speed < 0 && curr_d - dist > TOL))
		last_status = ONGOING;
	else
		last_status = SUCCESS;

	return last_status;
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
	if(angle > 0) //left turn
	{
		mDrive = mA_;
		mPivot = mB_;
		left_turn = true;
	}
	else // right turn
	{
		mDrive = mB_;
		mPivot = mA_;
		left_turn = false;
	}
	
	timeout = TIMEOUT_TOL*angle_/45 + millis(); // get timeout criteria
	speed = speed < 0 ? -speed : speed;   		// only work for forward turns currently
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
		mDrive->set_speed(speed);
		mPivot->set_speed(0);
		angle += imu->get_yaw(); //TODO make sure account for degrees popping back over
	}

	int32_t curr_angle = imu->get_yaw();
	int32_t curr_t = millis();

	//TODO complementary filter of encoder and imu values for current angle

	if(curr_t > timeout)
		last_status = FAILURE;
	else if(( left_turn && angle - curr_angle > TOL) ||
					( !left_turn && curr_angle - angle > TOL))
		last_status = ONGOING;
	else
		last_status = SUCCESS;

	return last_status;
}
