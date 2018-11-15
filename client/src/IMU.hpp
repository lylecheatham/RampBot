/* IMU.hpp
 * MTE 380 Design Project
 * Original Author(s): Eric Murphy-Zaremba 
 * Creation Date: Nov 14 /2018
 */

#ifndef IMU_HPP
#define IMU_HPP

#include <MPU9250.h>
#include "Arduino.h"
#include "constants.h"

class IMU {
public:
	IMU();
	~IMU();

	int32_t get_orientation();
	int32_t get_inclination();

private:
	static MPU9250FIFO *MPU;
	
	static char		print_buf[200];		// For debug and setup

	static float 	accel_data[3][85];
    static size_t 	accel_lengths[3];
    static float 	gyro_data[3][85];
    static size_t 	gyro_lengths[3];
    static float 	mag_data[3][85];
    static size_t 	mag_lengths[3];
    static float 	temp_data[85];
    static size_t 	temp_length;
	static int32_t 	status;

	static void read_IMU();


};



#endif
