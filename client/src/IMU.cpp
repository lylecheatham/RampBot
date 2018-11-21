/* IMU.cpp
 * MTE 380 Design Project
 * Original Author(s): Conner Currie & Eric Murphy-Zaremba
 * Creation Date: Nov 14 /2018
 *
 *
 * Implements the IMU class
 */


#include "IMU.hpp"

// Init MPU9250
MPU9250_DMP* IMU::imu = nullptr;
float IMU::pitch = 0;
float IMU::yaw = 0;
float IMU::roll = 0;

float IMU::pitch_prev = 0;
float IMU::yaw_prev = 0;
float IMU::roll_prev = 0;

IMU::IMU() {
    imu = new MPU9250_DMP();

    // Call imu.begin() to verify communication and initialize
    if (imu->begin() != INV_SUCCESS)
    {
        while (1)
        {
        Serial.println("Unable to communicate with MPU-9250");
        Serial.println("Check connections, and try again.");
        Serial.println();
        delay(5000);
        }
    }

    imu->dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
               FIFO_RATE); // Set DMP FIFO rate to 10 Hz
    // DMP_FEATURE_LP_QUAT can also be used. It uses the
    // accelerometer in low-power mode to estimate quat's.
    // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
    
}

IMU::~IMU() {
    delete imu;
}

float IMU::get_pitch() {
    if( abs(millis() - last_data_time) > 100){
        updateIMU();
    }
    return pitch;
}

float IMU::get_pitch_abs() {
    
    return imu->pitch;
}

float IMU::get_yaw() {
    if( abs(millis() - last_data_time) > 100){
        updateIMU();
    }
    return yaw;
}

float IMU::get_yaw_abs() {

    return imu->yaw;
}

float IMU::get_roll() {
    if( abs(millis() - last_data_time) > 100){
        updateIMU();
    }
    return roll;
}

float IMU::get_roll_abs() {

    return imu->roll;
}

void IMU::stabilize() {
	float prev_val;
	pinMode(GRN_LED, OUTPUT);
	digitalWrite(GRN_LED, 0); // Make sure LED off
	
	// Loop while it is drifting 
	do {
		prev_val = yaw;
		Serial.println(prev_val);
		delayMicroseconds(4000000); // delay two seconds	
	}while(abs(prev_val-yaw) > 0.05);
	Serial.println(yaw);

	digitalWrite(GRN_LED, 1); // signal value is good
}

void IMU::updateIMU()
{
    static bool LED_state = false;
    digitalWrite(13, LED_state);
    LED_state = !LED_state;

    //Get latest data reading
    imu->resetFifo();
    while(!imu->fifoAvailable()){};
    imu->updateFifo();

    //Get the time at which data was sampled
    last_data_time = millis();

    //Calculate euler angles from new data
    imu->computeEulerAngles();

    //For exceeding 360 yaw
    if ((imu->yaw - yaw_prev) < - 20){
        yaw += imu->yaw - yaw_prev + 360;
    }
    else if (imu->yaw - yaw_prev > 20){
        yaw += imu->yaw - yaw_prev - 360;
    }
    else{
        yaw += imu->yaw - yaw_prev;
    }

    //For exceeding 360 pitch
    if ((imu->pitch - pitch_prev) < - 20){
        pitch += imu->pitch - pitch_prev + 360;
    }
    else if (imu->pitch - pitch_prev > 20){
        pitch += imu->pitch - pitch_prev - 360;
    }
    else{
        pitch += imu->pitch - pitch_prev;
    }

    //For exceeding 360 roll
    if ((imu->roll - roll_prev) < - 20){
        pitch += imu->roll - roll_prev + 360;
    }
    else if (imu->roll - roll_prev > 20){
        pitch += imu->roll - roll_prev - 360;
    }
    else{
        pitch += imu->roll - roll_prev;
    }
}
