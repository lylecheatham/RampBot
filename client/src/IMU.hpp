/* IMU.hpp
 * MTE 380 Design Project
 * Original Author(s): Eric Murphy-Zaremba & Conner Currie
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
	static float 	yaw;
	static float 	pitch;
	static float 	roll;
	static float 		beta;								// 2 * proportional gain (Kp)
	static float 		q0;
	static float 		q1;
	static float 		q2;
	static float		q3;

	static float invSqrt(float x);
	static void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	static void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
	static void read_IMU();
	static void ComplementaryFilter();
	static void get_euler();
};



#endif
