/* All_Movements.hpp
 * 	MTE 380 Design Project
 * 	Original Author(s): Eric Murphy-Zaremba 
 * 	Creation Date: Nov 17 /2018
 *
 * 	Functionality:
 * 		This implements the abstract movement class into each subtype
 */ 

#ifndef ALL_MOVEMENTS_HPP
#define ALL_MOVEMENTS_HPP

#include "Movement.hpp" // Abstract parent class
#include "Motor.hpp"
#include "ultraSonicSwivel.h"
#include "IMU.hpp"
#include "constants.h"

/* 		Movements:
 * 			drive_sideways_scan
 * 			drive_distance
 * 			turn_angle
 * 			drive_onto_ramp
 * 			drive_over_ramp
 * 			drive_forward_angle_scan
 * 			hit_post_reorient
 */

#define TIMEOUT_TOL 30000 // timeout tolerance in us
#define TOL 5 // distance tolerance in cm

/* Drive Distance - (drive in a straight line a set distance) */
class DriveDistance : public Movement {
	public:
		DriveDistance(int32_t dist_, Motor* mA_, Motor* mB_, NewPingWrap* sonar_, IMU* imu_, int32_t speed_ = STD_SPEED);
		~DriveDistance() {};

		Status update();

	private:
		float k_p = 1.5;
		float start_ang;
		
		uint32_t timeout, prev_t, curr_t;
		int32_t encA_start, encB_start;
		int32_t dist;
	    int32_t	speedA, speedB, prev_speedA, prev_speedB, base_speed;
		Motor *mA, *mB;
		NewPingWrap *sonar;
		IMU *imu;

		float encoder_delta();
		//float imu_delta();

		void correct();
};


/* Turn Angle - (Turn to a specified angle off of current orientation / pivot turning) */
class TurnAngle : public Movement {
	public:
		TurnAngle(int32_t angle_, Motor* mA_, Motor* mB_, IMU* imu_, int32_t speed_ = STD_SPEED);
		~TurnAngle() {};

		Status update();

	private:
		// PID value
		float k_i = 0;
		float k_p = 1;
		float k_d = 0;
		float integration = 0;
		float prev_error = 1000000000;

		uint32_t timeout, prev_t, curr_t;
		int32_t start_enc;
		float angle, start_angle, prev_angle;
		int32_t speed, prev_speedD, prev_speedP;
		Motor *mDrive, *mPivot;
		IMU *imu;
		bool right_turn;		

		float encoder_angle();
		float imu_angle();

		void pid_update();
};

/* Drive Onto Ramp - (Drive in reverse, validate position between dist to boundary + accelerometer tilt + roll)*/
//class DriveOntoRamp : public Movement {
	//public:
		//DriveOntoRamp();
		//~DriveOntoRamp() {};

		//Status update();
	
	//private:
		//uint32_t timeout;

//};




#endif
