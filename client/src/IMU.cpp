/* IMU.cpp
 * MTE 380 Design Project
 * Original Author(s): Eric Murphy-Zaremba & Conner Currie
 * Creation Date: Nov 14 /2018
 *
 *
 * Implements the IMU class
 */
//extern "C"{
//#include "MadgwickAHRS.h"}

#include "IMU.hpp"
#include <math.h>

#define ACCELEROMETER_SENSITIVITY 8192.0
#define sampleFreq	50.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

// Init MPU9250 FIFO
MPU9250FIFO* IMU::MPU = nullptr;

//Init imu values
char		IMU::print_buf[200];
float 		IMU::accel_data[3][85];
size_t 		IMU::accel_lengths[3];
float  		IMU::gyro_data[3][85];
size_t 		IMU::gyro_lengths[3];
float  	 	IMU::mag_data[3][85];
size_t 		IMU::mag_lengths[3];
float  	 	IMU::temp_data[85];
size_t 		IMU::temp_length;
int32_t 	IMU::status;
float       IMU::yaw;
float 	    IMU::pitch;
float 	    IMU::roll;

float 		IMU::beta = betaDef;								// 2 * proportional gain (Kp)
float 		IMU::q0 = 1.0f;
float 		IMU::q1 = 0.0f;
float 		IMU::q2 = 0.0f;
float		IMU::q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

//elapsedMillis time;

IMU::IMU() {
	Wire.begin();
	MPU = new MPU9250FIFO(Wire, 0x68);
	status = MPU->begin();	

    snprintf(print_buf, 120, "Status: %d", status);
    Serial.println(print_buf);
    
    MPU->setAccelRange(MPU9250::AccelRange::ACCEL_RANGE_16G);
    snprintf(print_buf, 120, "Accel Range Success: %d", status);
    Serial.println(print_buf);

    MPU->setGyroRange(MPU9250::GyroRange::GYRO_RANGE_2000DPS);
    snprintf(print_buf, 120, "Gyro Range Success: %d", status);
    Serial.println(print_buf);

    MPU->enableFifo(true, true, true, true);

  	// setting DLPF bandwidth to 20 Hz
  	MPU->setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    
  	// setting SRD to 19 for a 50 Hz update rate
  	MPU->setSrd(19);

  	// enabling the data ready interrupt
  	MPU->enableDataReadyInterrupt();

  	// attaching the interrupt to microcontroller pin 1
  	pinMode(IMU_INT,INPUT);
  	attachInterrupt(IMU_INT,read_IMU,RISING);
}

IMU::~IMU() {
	delete MPU;	
}

void IMU::read_IMU() {
    //Set up LED 
    static bool LED_state = false;
    digitalWrite(13, LED_state);
    LED_state = !LED_state;

	status = MPU->readFifo();

    MPU->getFifoAccelX_mss(&accel_lengths[0], accel_data[0]);
    MPU->getFifoAccelY_mss(&accel_lengths[1], accel_data[1]);
    MPU->getFifoAccelZ_mss(&accel_lengths[2], accel_data[2]);
    MPU->getFifoGyroX_rads(&gyro_lengths[0], gyro_data[0]);
    MPU->getFifoGyroY_rads(&gyro_lengths[1], gyro_data[1]);
    MPU->getFifoGyroZ_rads(&gyro_lengths[2], gyro_data[2]);
    MPU->getFifoMagX_uT(&mag_lengths[0], mag_data[0]);
    MPU->getFifoMagY_uT(&mag_lengths[1], mag_data[1]);
    MPU->getFifoMagZ_uT(&mag_lengths[2], mag_data[2]);
    MPU->getFifoTemperature_C(&temp_length, temp_data);
    
    //snprintf(print_buf, 200, "s:%-3d, ax:%-3u, gx:%-3u, mx:%-3u, t:%-3u, AX:%+5f, GX:%+5f, MX:%+5f, T:%+5f", 
    //         status, accel_lengths[0], gyro_lengths[0],
    //         mag_lengths[0], temp_length, accel_data[0][0], gyro_data[0][0], mag_data[0][0], temp_data[0]);

    MadgwickAHRSupdate(gyro_data[0][0],gyro_data[1][0],gyro_data[2][0],
                            accel_data[0][0], accel_data[1][0], accel_data[2][0],
                            mag_data[0][0], mag_data[1][0], mag_data[2][0]);
	//Update Euler values
	get_euler();
	//Print Values
    snprintf(print_buf, 200, "Yaw:%-3u, Pitch:%-3d, Roll:%-3u", yaw, pitch, roll);
    Serial.println(print_buf);
    //Serial.println(time);
}

void IMU::ComplementaryFilter()
{
    //Inputs: short accData[3], short gyrData[3], float *pitch, float *roll
    float dt = 0.02;
    float GYROSCOPE_SENSITIVITY = 65.536;
    float pitchAcc, rollAcc;               
    
    pitchAcc = gyro_data[0][0];
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    pitch += (gyro_data[0][0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
    roll -= (gyro_data[1][0] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
 
    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
    int forceMagnitudeApprox = abs(accel_data[0][0]) + abs(accel_data[1][0]) + abs(accel_data[2][0]);

    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {

	// Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f(accel_data[1][0], accel_data[2][0]) * 180 / M_PI;
        pitch = pitch * 0.98 + pitchAcc * 0.02;
 
	// Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f(accel_data[0][0], accel_data[2][0]) * 180 / M_PI;
        roll = roll * 0.98 + rollAcc * 0.02;
    }
} 

void IMU::MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void IMU::MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float IMU::invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void IMU::get_euler(){
    double test = q1*q2 + q3*q0;
	if (test > 0.499) { // singularity at north pole
		yaw = 2 * atan2(q1,q0);
		pitch = M_PI/2;
		roll = 0;
		return;
	}
	if (test < -0.499) { // singularity at south pole
		yaw = -2 * atan2(q1,q0);
		pitch = - M_PI/2;
		roll = 0;
		return;
	}
    double sqx = q1*q1;
    double sqy = q2*q2;
    double sqz = q3*q3;

    yaw = atan2(2*q2*q0-2*q1*q3 , 1 - 2*sqy - 2*sqz);
	pitch = asin(2*test);
	roll = atan2(2*q1*q0-2*q2*q3 , 1 - 2*sqx - 2*sqz);
}
//====================================================================================================
// END OF CODE
//====================================================================================================