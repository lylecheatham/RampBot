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

#include <cstdint>

enum Status {
	TIMEOUT = 0,
	SUCCESS,
	INIT
};

class Movement {
	public:
		Status last_status;

		Movement() {};
		~Movement() {};

		virtual Status run() = 0;

	protected:
		// Members --
		// P Control
		float k_p;
		int32_t  error, base_speed; 

		// Timeout 
		uint32_t timeout;

		// For maintaining bearing
		float target_angle, curr_angle;

		// Functions --
		// P-control for bearing
		int32_t p_control() 
		{ 
			error = curr_angle-target_angle;
			return static_cast<int32_t>(k_p*error); 
		}	
};

#endif 
