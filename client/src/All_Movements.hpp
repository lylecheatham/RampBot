/* All_Movements.hpp
 * 	 Design Project
 * 	Original Author(s): Eric Murphy-Zaremba
 * 	Creation Date: Nov 17 /2018
 *
 * 	Functionality:
 * 		This implements the abstract movement class into each subtype
 */

#ifndef ALL_MOVEMENTS_HPP
#define ALL_MOVEMENTS_HPP

#include "Movement.hpp"  // Abstract parent class
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

#define TIMEOUT_TOL 30000  // timeout tolerance in us
#define TOL 1              // distance tolerance in cm

/* Drive Distance - (drive in a straight line a set distance) */
class DriveDistance : public Movement {
public:
    DriveDistance(int32_t dist_, int32_t speed_ = STD_SPEED);
    ~DriveDistance(){};

    Status run(Robot &robot);

protected:
    int32_t encA_start, encB_start;
    int32_t travel_distance;
    int32_t speedA, speedB;

    virtual void init(Robot &robot);
    virtual bool success(Robot &robot);
    virtual bool continue_run(Robot &robot);
    virtual void clean(Robot &robot);
    virtual int32_t get_dist(Robot &robot);	
    float encoder_dist_cm(Robot &robot);

private:
    float encoder_delta(Robot &robot);  // Get difference between encoder values (in case unable to use imu)
};

/* DriveDistanceSonar - Same as DriveDistance but using sonar */
class DriveDistanceSonar : public DriveDistance {
public:
    DriveDistanceSonar(int32_t dist, int32_t speed_ = STD_SPEED);
	~DriveDistanceSonar(){};

protected:
	void init(Robot &robot);
	int32_t get_dist(Robot &robot);

private:
	int32_t start_dist;
};


/* Turn Angle - (Turn to a specified angle off of current orientation / pivot turning) */
class TurnAngle : public Movement {
public:
    TurnAngle(float angle_, bool forward_turn_ = true);
    ~TurnAngle(){};

    Status run(Robot &robot);

private:
    int32_t start_enc;
    float turn_angle;
	bool forward_turn;

    Motor *mDrive, *mPivot;

    bool right_turn;
};

/*
 *class TurnAbsolute : public Movement {
 *public:
 *    TurnAbsolute(Angle angle_);
 *    ~TurnAbsolute(){};
 *
 *    Status run(Robot &robot);
 *
 *private:
 *    Angle turn_angle;
 *
 *    Motor *mDrive, *mPivot;
 *
 *    bool right_turn;
 *};
 */

/* Drive Onto Ramp - (Drive in reverse, validate position between dist to boundary + accelerometer tilt + roll)*/
class RampMovement : public Movement {
public:
    RampMovement();
    ~RampMovement(){};

    Status run(Robot &robot);

private:
    uint32_t timeout;
};

/* Find Post */
class FindPost : public DriveDistance {
public:
    FindPost(int32_t search_dist, int32_t travel_dist, int32_t speed_ = STD_SPEED);
    ~FindPost(){};

protected:
    void init(Robot &robot);
    bool success(Robot &robot);
    bool continue_run(Robot &robot);
    void clean(Robot &robot);

private:
    const int32_t servo_angle = 175;
    const int32_t post_tol = 10;
    int32_t servo_start, search_distance, cm_start;
};

/* Run to Post */
class DriveToPost : public DriveDistance {
public:
    DriveToPost(int32_t dist_, int32_t speed_ = STD_SPEED);
    ~DriveToPost(){};

protected:
	void init(Robot &robot);
    bool success(Robot &robot);
    bool continue_run(Robot &robot);

private:
	uint32_t start_time;
	const uint32_t delay_time = 2000;
    const float angle_tol = 5;
	const float accel_tol = -0.4;
};


/* Correct Ramp */

/* Lateral Shift */

/*
 *class LateralShift : public Movement {
 *    public:
 *        LateralShift(int32_t shift_, int32_t speed_ = STD_SPEED);
 *        ~LateralShift() {};
 *
 *        Status run(Robot &robot);
 *
 *    private:
 *   	 	int32_t encA_start, encB_start;
 *   	 	int32_t shift;
 *   	 	int32_t speedA, speedB;
 *
 *   	 	Motor *mA, *mB;
 *   	 	UltraSonicSwivel* servo;
 *   	 	IMU* imu;
 *};
 */



#endif
