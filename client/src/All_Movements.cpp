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
		travel_distance(dist_),
		mA(mA_),
		mB(mB_),
		servo(servo_),
		imu(imu_)
{
	k_p = 1.5;
	k_i = 0.001;
	k_d = 0.001;
	freq = 100;
	integration = 0;

	//timeout = 1000*dist_/(speed_*RPM_TO_VO) + TIMEOUT_TOL + millis(); // Get timeout criteria
	base_speed = travel_distance*speed_ < 0 ? -speed_ : speed_;                  // Account for direction
	speedA = 0;
	speedB = 0;
	last_status = INIT;
}

/* run
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		current execution status (SUCCESS / FAILURE / ONGOING)
 */
Status DriveDistance::run()
{
	init();

	// Speed adjustment
	int32_t speed_adj = 0;
	uint32_t curr_t = 0;

	// Loop until timed out
	while(continue_run()){ 
		curr_angle = imu->get_yaw();
		while(millis()-curr_t < 10); // only update at 100Hz
		speed_adj = pid_control(); 		
		curr_t = millis();
		speedA = base_speed + speed_adj;
		speedB = base_speed - speed_adj;

		mA->set_speed(speedA);
		mB->set_speed(speedB);

		if(success())
			break;
	}

	clean();

	return curr_t > timeout ? TIMEOUT : SUCCESS;
}


/* init 
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		none
 */
void DriveDistance::init()
{
	// Ensure motors stopped to begin with
	mA->set_speed(speedA);
	mB->set_speed(speedB);

	// Initial Conditions
	encA_start  = mA->get_count();
	encB_start  = mB->get_count();
	
	//travel_distance = abs(travel_distance) < 5 ? 5 : travel_distance;	    //Ensure no negative distances	
	target_angle = imu->get_yaw();
	
	timeout = millis() + 1000000; // TODO maybe improve
}


/* update
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		Whether success criteria met (true/false)
 */
bool DriveDistance::success() 
{
		int32_t curr_dist = get_dist();
		Serial.println(speedA);
		Serial.println(speedB);
		Serial.println(travel_distance);
		Serial.println(curr_dist < travel_distance);
		Serial.print("Curr d: ");
		Serial.println(curr_dist);
		return (speedA > 0 && speedA*speedB > 0 && curr_dist > travel_distance) || 
				(speedA < 0 && speedA*speedB > 0 && curr_dist < travel_distance);
}

/* continue_run 
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		Whether should continue running (checks timeout) (true/false)
 */
bool DriveDistance::continue_run() 
{
	return millis() < timeout;
}	

/* clean 
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		none
 */
void DriveDistance::clean()
{
	mA->set_speed(0); // ensure motor stopped at this point
	mB->set_speed(0);
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
//	return servo->sensor.ping_cm();
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
		while(millis()-curr_t < 1); // only update at 100Hz
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
	//Initialize variables
	float ramp_speed[4] = {20,20,20,20};
	float imu_state[4] = {-30,0,30,0};
	int ramp_state = 0;
	int tolerance = 3; 
	uint32_t curr_t = 0;

	//Set initial speed
	mA->set_speed(-ramp_speed[ramp_state]);
	mB->set_speed(-ramp_speed[ramp_state]);

	//Carry out ramp movement
	while(ramp_state < 4){
		
		if (abs(imu->get_pitch() - imu_state[ramp_state]) < tolerance){
			Serial.println(imu->get_pitch());
			mA->set_speed(-ramp_speed[ramp_state]);
			mB->set_speed(-ramp_speed[ramp_state]);

			//Get time:
			curr_t = millis();
			Serial.println("State Transition");
			ramp_state += 1;
		}
	}
	Serial.println("End of Ramp");

	return curr_t > timeout ? TIMEOUT : SUCCESS;
}



/********************* Find Post ***************************/
/* FindPost
 * 	Inputs:
 * 		search_dist  : the desired search radius for the post (0 uses the first reading of the sonar)
 *		travel_dist  : desired distance to travel (+ for forwards, - for backwards)
 *		mA_    : pointer to the right motor (A)
 *		mb_    : pointer to the left motor (B)
 *		sonar_ : pointer to the ultrasonic sensor object
 *		speed_ : (optional) desired motor speed for the action
 * 	Outputs:
 * 		none
 */
FindPost::FindPost(int32_t search_dist, int32_t travel_dist, Motor* mA_, Motor* mB_, UltraSonicSwivel* servo_, IMU* imu_, int32_t speed_) :
	search_distance(search_dist),
	DriveDistance(travel_dist, mA_, mB_, servo_, imu_, speed_)
{
}

/* init 
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		none
 */
void FindPost::init()
{
	// Ensure motors stopped to begin with
	mA->set_speed(speedA);
	mB->set_speed(speedB);

	// Set heading
	target_angle = imu->get_yaw();	

	// Initial Conditions
	encA_start  = mA->get_count();
	encB_start  = mB->get_count();
	servo_start = servo->get_position();
	servo->set_position(servo_angle);
	delay(500);

	cm_start = servo->sensor.ping_cm(); 
}


/* success
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		Whether success criteria met (true/false)
 */
bool FindPost::success() 
{
	return search_distance > 0 ? 
			servo->sensor.ping_cm() < search_distance - post_tol : 
			servo->sensor.ping_cm() < cm_start - post_tol;
}

/* continue_run 
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		Whether should continue running (checks timeout) (true/false)
 */
bool FindPost::continue_run() 
{
	return millis() < timeout && encoder_dist_cm() < travel_distance;
}	

/* clean 
 * 	Inputs:
 * 		none
 * 	Outputs:
 * 		none
 */
void FindPost::clean()
{
	servo->set_position(servo_start);
	mA->set_speed(0); // ensure motor stopped at this point
	mB->set_speed(0);
}


