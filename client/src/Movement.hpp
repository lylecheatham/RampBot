/* Movement.hpp
 * 	MTE 380 Design Project
 * 	Original Author(s): Lyle Cheatham / Eric Murphy-Zaremba 
 * 	Creation Date: Nov 16 /2018
 *
 * 	Functionality:
 * 		This abstract class is to be used for each type of movement associated with the robot 
 * 		of the form: action + ending condition.
 *
 * 		Movements:
 * 			drive_forward_scan
 * 			drive_sideways_scan
 * 			drive_distance
 * 			turn_angle
 * 			drive_onto_ramp
 * 			drive_over_ramp
 * 			drive_forward_angle_scan
 * 			hit_post_reorient
 */

#ifndef MOVEMENT_HPP
#define MOVEMENT_HPP

enum Status {
	FAILURE = 0,
	SUCCESS,
	ONGOING
};

class Movement {
	public:
		Status last_status;

		Movement();
		~Movement();

		virtual Status update() = 0;	
};

#endif 
