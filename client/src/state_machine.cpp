/* state_machine.cpp
 * MTE 380 Design Project
 * Original Author: Conner Currie
 * Creation Date: 11/12/2018
 *
 * Class containing functions for traversing the course
 */

#include "state_machine.hpp"
#include "All_Movements.hpp"

// #define SIDE_A_PLATFORM_TO_RAMP_ALIGN
// #define RAMP_ALIGN
// #define RAMP_RUN
// #define SIDE_B_RAMP_END
#define SIDE_B_FIND_POST
//#define SIDE_B_RAMP_END
//#define RAMP_S_TURN

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
        TurnAngle turnL(-90);
        TurnAngle turnR(90);

        //#########################################################################
        // PHASE 1 - ASSUME ROBOT IS FACING THE BACK BOUNDARY WALL
        //#########################################################################

        robot.imu.compensate_pitch(1,0);
        robot.imu.compensate_roll(1,0);
        robot.imu.compensate_yaw(1,0);
        robot.target_angle = 0;

#ifdef SIDE_A_PLATFORM_TO_RAMP_ALIGN
        // Get distance to the back wall and feed that into DriveDistance
        // Note - subtract the length of the robot from this (18cm plus 5cm tol)
        DriveDistance fwd_1(robot.swivel.sensor.ping_cm() - 35, 80);
        execute(fwd_1);

        // Take a left turn
        execute(turnL);

        // Get distance to the side wall and feed that into DriveDistance
        // Note - subtract the distance to the ramp from this (FIND VALUE)
        DriveDistance fwd_2(robot.swivel.sensor.ping_cm() - 42, 80);
        execute(fwd_2);

        // Take a right turn
        execute(turnR);
#endif // SIDE_A_PLATFORM_TO_RAMP_ALIGN

        //Ramp Alignment
#ifdef RAMP_ALIGN

        //Take ultrasonic sample
        uint32_t start_dist = robot.swivel.sensor.ping_cm();
        robot.swivel.set_position(178);

        float alignment_angle;
        float dist_meas = robot.swivel.sensor.ping_cm();
        alignment_angle = atan2(33 - dist_meas, 70-start_dist)*180/M_PI;
        alignment_angle = alignment_angle > 90 ? alignment_angle -180 : alignment_angle;
        Serial.print("Angle: ");
        Serial.println(alignment_angle);

        //Execute Turn
        TurnAngle ramp_alignment(alignment_angle);
        execute(ramp_alignment);

#endif // RAMP_ALIGN

#ifdef RAMP_S_TURN
        robot.imu.compensate_pitch(1,0);
        robot.imu.compensate_roll(1,0);
        robot.imu.compensate_yaw(1,0);

		robot.swivel.set_position(178);
		delay(1000);


       float dist_meas = robot.swivel.sensor.ping_cm();

       float shift = dist_meas - 33; // +'ve right turn -'ve left turn
       float angle = acos(1 - abs(shift)*1.0/WHEELBASE_CM)*180/M_PI;
       angle = shift < 0 ? angle : -angle;

       Serial.print("Angle: ");
       Serial.println(angle);

       robot.swivel.set_position(90);

       TurnAngle s_turn(angle, false);
       TurnAngle s_turn_back(-angle, false);

       execute(s_turn);
       execute(s_turn_back); 
#endif // RAMP_S_TURN

#ifdef RAMP_RUN
        // Carry out ramp movement
        RampMovement ramp;
        execute(ramp);

#endif // RAMP_RUN

#ifdef SIDE_B_FIND_POST
        //Move a bit forward
        Serial.println("HERE");
        DriveDistance post_ramp(-40, 40);
        Serial.println(execute(post_ramp));
        

        //delay(1000);

        // Take a right turn
        execute(turnR);

        //Check to see if post is in front
        // if(robot.swivel.sensor.ping_cm() < 195){
        //     // // Touch the post
        //     robot.imu.compensate_pitch(1,0);
        //     robot.imu.compensate_roll(1,0);
        //     DriveToPost drive_post(200, 100);
        //     execute(drive_post);
        // }
        // else{
            // Move past the ramp to avoid pinging it
            // Assumed 20cm - FIND THIS LATER
            DriveDistance fwd_3(20, 20);
            execute(fwd_3);

            // Carry out post detection algorithm - first attempt:

            // Find post
            // FIND SEARCH DISTANCE EXPERIMENTALLY

            // Status result;
            // int attempt = 0;
            // while (1) {
            //     FindPost search(200, 150 - attempt * 10, 140);

            //     result = execute(search);

            //     if (result == SUCCESS) {
            //         break;
            //     }

            //     else if (result == TIMEOUT) {
            //         attempt += 1;
            //         execute(turnL);
            //     }
            // }

            FindPost search(200, 150, 140);

            execute(search);
            // Face post
            TurnAngle post_turn(-80);
            execute(post_turn);

            robot.imu.compensate_pitch(1,0);
            robot.imu.compensate_roll(1,0);

            // // Touch the post
            DriveToPost drive_post(200, 100);
            Serial.println(execute(drive_post));
        // }

#endif // SIDE_B_FIND_POST

        // // To be implemented by Eric

        // //#########################################################################
        // // PHASE 2 - THE RETURN
        // //#########################################################################

        // // Turn to face the back
        // // Take a left turn
        // execute(turnL);
        // // Take a left turn
        // execute(turnL);

        // // Get distance to the back wall and feed that into DriveDistance
        // // Note - subtract the length of the robot from this (18cm plus 5cm tol)
        // DriveDistance fwd_5(robot.swivel.sensor.ping_cm() - 25, 20);
        // execute(fwd_5);

        // // Take a right turn
        // execute(turnR);

        // // Get distance to the side wall and feed that into DriveDistance
        // // Note - subtract the distance to the ramp from this (FIND VALUE)
        // DriveDistance fwd_6(robot.swivel.sensor.ping_cm() - 25, 20);
        // execute(fwd_6);

        // // Take a left turn
        // execute(turnL);

        // // Carry out ramp movement
        // execute(ramp);

        // // Take a right turn
        // execute(turnL);

        // // Carry out first distances in inverted order
        // execute(fwd_2);
        // execute(turnR);
        // execute(fwd_1);

        // Finish
    }
}


/* function: execute
 * 		Executes a given movement
 */
Status state_machine::execute(Movement& m) {
    if (get_pushbutton()) {
        robot.imu.compensate_yaw(1, Angle(0));
        robot.target_angle = Angle(0);
    }
    return m.run(robot);
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
