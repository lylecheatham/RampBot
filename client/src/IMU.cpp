/* IMU.cpp
 * MTE 380 Design Project
 * Original Author(s): Eric Murphy-Zaremba 
 * Creation Date: Nov 14 /2018
 *
 *
 * Implements the IMU class
 */


#include "IMU.hpp"


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

    snprintf(print_buf, 200, "s:%-3d, ax:%-3u, gx:%-3u, mx:%-3u, t:%-3u, AX:%+5f, GX:%+5f, MX:%+5f, T:%+5f", status, accel_lengths[0], gyro_lengths[0],
             mag_lengths[0], temp_length, accel_data[0][0], gyro_data[0][0], mag_data[0][0], temp_data[0]);

}
