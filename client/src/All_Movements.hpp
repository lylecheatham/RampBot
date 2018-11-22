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

#include "IMU.hpp"
#include "Motor.hpp"
#include "Movement.hpp"  // Abstract parent class
#include "constants.h"
#include "ultraSonicSwivel.h"

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
    DriveDistance(int32_t dist_, Motor* mA_, Motor* mB_, UltraSonicSwivel* servo_, IMU* imu_, int32_t speed_ = STD_SPEED);
    ~DriveDistance(){};

    Status run();

protected:
    int32_t encA_start, encB_start;
    int32_t travel_distance;
    int32_t speedA, speedB;

    Motor *mA, *mB;
    UltraSonicSwivel* servo;
    IMU* imu;

    virtual void init();
    virtual bool success();
    virtual bool continue_run();
    virtual void clean();
    float encoder_dist_cm();

private:
    int32_t get_dist();
    float encoder_delta();  // Get difference between encoder values (in case unable to use imu)
};


/* Turn Angle - (Turn to a specified angle off of current orientation / pivot turning) */
class TurnAngle : public Movement {
public:
    TurnAngle(float angle_, Motor* mA_, Motor* mB_, IMU* imu_);
    ~TurnAngle(){};

    Status run();

private:
    int32_t start_enc;
    float turn_angle;

    Motor *mDrive, *mPivot;
    IMU* imu;

    bool right_turn;
};

/* Drive Onto Ramp - (Drive in reverse, validate position between dist to boundary + accelerometer tilt + roll)*/
class RampMovement : public Movement {
public:
    RampMovement(Motor* mA_, Motor* mB_, IMU* imu_);
    ~RampMovement(){};

    Status run();

private:
    Motor *mA, *mB;
    IMU* imu;
    uint32_t timeout;
};

/* Find Post */
class FindPost : public DriveDistance {
public:
    FindPost(int32_t search_dist, int32_t travel_dist, Motor* mA_, Motor* mB_, UltraSonicSwivel* servo_, IMU* imu_, int32_t speed_ = STD_SPEED);
    ~FindPost(){};

protected:
    void init();
    bool success();
    bool continue_run();
    void clean();

private:
    const int32_t servo_angle = 175;
    const int32_t post_tol = 10;
    int32_t servo_start, search_distance, cm_start;
};

/* Correct Ramp */
/*
 *class RampCorrect : public Movement {
 *    public:
 *        RampCorrect();
 *        ~RampCorrect() {};
 *
 *        Status run();
 *
 *    private:
 *
 *
 *
 *};
 */

#endif
