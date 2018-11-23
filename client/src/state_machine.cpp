/* state_machine.cpp
 * MTE 380 Design Project
 * Original Author: Conner Currie
 * Creation Date: 11/12/2018
 *
 * Class containing functions for traversing the course
 */

#include "state_machine.hpp"
#include "All_Movements.hpp"

/////#define SIDE_A_PLATFORM_TO_RAMP_ALIGN
//  #define RAMP_ALIGN
/////#define RAMP_RUN
// #define SIDE_B_RAMP_END
/////#define SIDE_B_FIND_POST
//#define SIDE_B_RAMP_END
////#define RAMP_S_TURN
#define TO_S_TURN
//#define RAMP_S_TURN_2
#define RAMP_TWO
#define FINISH_COURSE

// Phase 1
/*
 *#define SIDE_A_PLATFORM_TO_RAMP_ALIGN
 *#define RAMP_RUN
 *#define SIDE_B_FIND_POST
 *#define RAMP_S_TURN
 */

//Phase 2
/*
 *#define TO_S_TURN
 *#define RAMP_TWO
 *#define FINSIH_COURSE
 */

elapsedMillis time;

bool state_machine::button_flag = false;

/* function: state_machine()
 *
 *
 *
 */
state_machine::state_machine() {
    robot.init();
    attachInterrupt(20, button_interrupt, RISING);
}


/* function: ~state_machine()
 *
 *
 */
state_machine::~state_machine() {}

inline void state_machine::get_dist(int32_t& dist) {
    dist = robot.swivel.sensor.ping_cm();
    dist = dist == 0 ? 300 : dist;
}


/* function: start()
 *
 * 	 	Function for initially finding the position of the robot on the course
 *
 *
 */
void state_machine::start() {
    while (1) {

        //Starting Logic
        Serial.print("Wave hand in front of sensor to continue");
        while(1){
            Serial.print(".");
            delay(100);
            if (robot.swivel.sensor.ping_cm() < 5){
                break;
            }
        }
        //Give time to remove hand
        delay(1000);

        // Initialize left and right turns
        TurnAngle* turnL = new TurnAngle(-90);
        TurnAngle* turnR = new TurnAngle(90);

        //#########################################################################
        // PHASE 1 - ASSUME ROBOT IS FACING THE BACK BOUNDARY WALL
        //#########################################################################

        robot.imu.compensate_pitch(1,0);
        robot.imu.compensate_roll(1,0);
        robot.imu.compensate_yaw(1,0);
        robot.target_angle = 0;

#ifdef SIDE_A_PLATFORM_TO_RAMP_ALIGN
{
        // Get distance to the back wall and feed that into DriveDistance
        // Note - subtract the length of the robot from this (18cm plus 5cm tol)

        uint32_t sample = 35;
		while(sample <= 35)
			sample = robot.swivel.sensor.ping_cm();

        DriveDistance* fwd_1 = new DriveDistance(sample - 35, 80);

        execute(fwd_1);
        delete fwd_1;
        // Take a left turn
        execute(turnL);

        // Get distance to the side wall and feed that into DriveDistance
        // Note - subtract the distance to the ramp from this (FIND VALUE)
        sample = robot.swivel.sensor.ping_cm();
        DriveDistance* fwd_2 = new DriveDistance(sample - 42, 80);
        execute(fwd_2);
        delete fwd_2;
        // Take a right turn
        execute(turnR);
        
}
#endif // SIDE_A_PLATFORM_TO_RAMP_ALIGN

#ifdef RAMP_S_TURN
{
       // robot.imu.compensate_pitch(1,0);
        //robot.imu.compensate_roll(1,0);
        //robot.imu.compensate_yaw(1,0);

		robot.swivel.set_position(178);
		delay(1000);


       float dist_meas = robot.swivel.sensor.ping_cm();

       float shift = dist_meas - 33; // +'ve right turn -'ve left turn
       float angle = acos(1 - abs(shift)*1.0/WHEELBASE_CM)*180/M_PI;
       angle = shift < 0 ? angle : -angle;

       Serial.print("Angle: ");
       Serial.println(angle);

       robot.swivel.set_position(90);

       TurnAngle* s_turn = new TurnAngle(angle, false);
       TurnAngle* s_turn_back = new TurnAngle(-angle, false);

       execute(s_turn);
       execute(s_turn_back); 
       delete s_turn;
       delete s_turn_back;
}
#endif // RAMP_S_TURN
{
	   DriveDistance* driving = new DriveDistance(-10, 20);
	   execute(driving);
       delete driving;
}
#ifdef RAMP_RUN
{
        // Carry out ramp movement
        RampMovement* ramp = new RampMovement;
        execute(ramp);
        delete ramp;
}
#endif // RAMP_RUN

#ifdef SIDE_B_FIND_POST
{
        //Move a bit forward
        Serial.println("HERE");
        DriveDistance* post_ramp = new DriveDistance(-60, 40);
        execute(post_ramp);
        delete post_ramp;

		robot.imu.compensate_pitch(1,0);
		robot.imu.compensate_roll(1,0);
        robot.imu.compensate_yaw(1,0);
        robot.target_angle = 0;

        //delay(1000);

        // Take a right turn
        execute(turnR);

        
        DriveDistance* fwd_3 = new DriveDistance(40, 60);
        execute(fwd_3);
        delete fwd_3;
        FindPost* search = new FindPost(0, 150, 140);

        execute(search);
        delete search;
        // Face post
        TurnAngle* post_turn = new TurnAngle(-75);
        execute(post_turn);
        delete post_turn;

        robot.imu.compensate_pitch(1,0);
        robot.imu.compensate_roll(1,0);

        // // Touch the post
        DriveDistance* drive_post = new DriveDistance(200, 100);
        Serial.println(execute(drive_post));
}
#endif // SIDE_B_FIND_POST

#ifdef TO_S_TURN
{
        //#########################################################################
        // PHASE 2 - THE RETURN
        //#########################################################################
/*
        //Reverse from original position, get back to 80 degree heading
        DriveDistance* reverse_from_post = new DriveDistance(-40, 40);
        execute(reverse_from_post);
        delete reverse_from_post;
        //Angle robot back to face the wall
        TurnAngle* post_turn_revert = new TurnAngle(165);
        execute(post_turn_revert);
        delete post_turn_revert;
*/
        // Get distance to the back wall and feed that into DriveDistance
        // Note - subtract the length of the robot from this (18cm plus 5cm tol)
        //uint32_t sample = robot.swivel.sensor.ping_cm();
        DriveDistance* fwd_5 = new DriveDistance(robot.swivel.sensor.ping_cm() -35, 100);
        execute(fwd_5);
        delete fwd_5;

/*        DriveDistance* back_off = new DriveDistance(-33, 40);
        execute(back_off);
        delete back_off;
*/
        // Take a right turn
        execute(turnR);
		
        // Get distance to the side wall and feed that into DriveDistance
        // Note - subtract the distance to the ramp from this (FIND VALUE)
        //sample = robot.swivel.sensor.ping_cm();
        DriveDistance* fwd_6 = new DriveDistance(robot.swivel.sensor.ping_cm() -29, 80);
        execute(fwd_6);
        delete fwd_6;
/*
        DriveDistance* back_off_again = new DriveDistance(-28, 40);
        execute(back_off);
        delete back_off_again;

*/
        TurnAngle* turnLAGAIN = new TurnAngle(-90);
        // Take a left turn
        execute(turnLAGAIN);
        delete turnLAGAIN;
}
#endif

#ifdef RAMP_S_TURN_2
{
       // robot.imu.compensate_pitch(1,0);
        //robot.imu.compensate_roll(1,0);
        //robot.imu.compensate_yaw(1,0);

		robot.swivel.set_position(1);
		delay(1000);


       float dist_meas = robot.swivel.sensor.ping_cm();

       float shift = dist_meas - 25; // +'ve right turn -'ve left turn
       float angle = acos(1 - abs(shift)*1.0/WHEELBASE_CM)*180/M_PI;
       angle = shift > 0 ? angle : -angle;

       Serial.print("Angle: ");
       Serial.println(angle);

       robot.swivel.set_position(90);

       TurnAngle* s_turn_2 = new TurnAngle(angle, false);
       TurnAngle* s_turn_back_2 = new TurnAngle(-angle, false);

       execute(s_turn_2);
       execute(s_turn_back_2); 
       delete s_turn_2;
       delete s_turn_back_2;
}

#endif // RAMP_S_TURN_2

#ifdef RAMP_TWO
{
        // //Drive forward
        // execute(driving);

        // // Carry out ramp movement
        RampMovement* ramp_two = new RampMovement;
        execute(ramp_two);
        delete ramp_two;
        // // Take a right turn
        // execute(turnL);

        // // Carry out first distances in inverted order
        // execute(fwd_2);
        // execute(turnR);
        // execute(fwd_1);

        // Finish
}
#endif

#ifdef FINISH_COURSE
{
    DriveDistance* reverse_from_post_again = new DriveDistance(-40, 40);
    execute(reverse_from_post_again);
    delete reverse_from_post_again;
    execute(turnL);

    DriveDistance* to_start = new DriveDistance(80, 60);
    execute(to_start);
    delete to_start;
    execute(turnR);

    DriveDistance* to_end = new DriveDistance(50, 60);
    execute(to_end);
    delete to_end;
}

#endif
    }
}


/* function: execute
 * 		Executes a given movement
 */
Status state_machine::execute(Movement* m) {
    return m->run(robot);
}

/* function: get_char()
 *
 */
int8_t state_machine::get_char() {
    if (Serial.available() > 0) return Serial.read();
    return -1;
}


/* function: stop()
 *
 */
void state_machine::stop() {
    robot.mA.set_speed(0);
    robot.mB.set_speed(0);
}

void state_machine::button_interrupt() {
    button_flag = true;
}

bool state_machine::get_pushbutton() {
    if (button_flag) {
        button_flag = false;
        return true;
    }
    return false;
}
