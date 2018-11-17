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

#define TIMEOUT_TOL 3000 // timeout tolerance in ms
#define TOL 1 // distance tolerance in cm

/* Drive Distance - (drive in a straight line a set distance) */
class DriveDistance : Movement {
	public:
		DriveDistance(int32_t dist_, Motor* mA_, Motor* mB_, NewPing* sonar_, int32_t speed_ = STD_SPEED);
		~DriveDistance() {};

		Status update();

	private:
		uint32_t timeout;
		int32_t dist;
	    int32_t	speed, prev_speedA, prev_speedB;
		Motor *mA, *mB;
		NewPing *sonar;

		void correct();
};


/* Turn Angle - (Turn to a specified angle off of current orientation / pivot turning) */
class TurnAngle : Movement {
	public:
		TurnAngle(int32_t angle_, Motor* mA_, Motor* mB_, IMU* imu_, int32_t speed_ = STD_SPEED);
		~TurnAngle() {};

		Status update();

	private:
		uint32_t timeout, prev_t;
		int32_t start_enc;
		int32_t angle, start_angle, prev_angle;
		int32_t speed, prev_speedD, prev_speedP;
		Motor *mDrive, *mPivot;
		IMU *imu;
		bool left_turn;		

		int32_t encoder_angle();
		int32_t imu_angle();
};

/* Drive Onto Ramp - (Drive in reverse, validate position between dist to boundary + accelerometer tilt + roll)*/
//class DriveOntoRamp : Movement {
	//public:
		//DriveOntoRamp();
		//~DriveOntoRamp() {};

		//Status update();
	
	//private:
		//uint32_t timeout;

//};




#endif
