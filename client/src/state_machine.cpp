/* state_machine.cpp
 * MTE 380 Design Project
 * Original Author: Conner Currie
 * Creation Date: 11/12/2018
 *
 * Class containing functions for traversing the course
 */

#include "state_machine.hpp"
#include "ultraSonicSwivel.h"
#include "DemoInterface.hpp"


//center the servo
int state_machine::servo_pos = 90;

Motor* state_machine::mA = nullptr;
Motor* state_machine::mB = nullptr;

int32_t state_machine::speedA = 0;
int32_t state_machine::speedB = 0;

std::string state_machine::error_string = "";

// Initialize swivel Variable
UltraSonicSwivel* state_machine::servo = nullptr;

// Initialize IMU
MPU9250FIFO* state_machine::IMU = nullptr;

//Initialize the Ultrasonic Sensor - (pin,pin,max)
NewPing sonar(U_PING, U_PING, 300);

//Init imu values
char		state_machine::print_buf[200];
float 		state_machine::accel_data[3][85];
size_t 		state_machine::accel_lengths[3];
float  		state_machine::gyro_data[3][85];
size_t 		state_machine::gyro_lengths[3];
float  	 	state_machine::mag_data[3][85];
size_t 		state_machine::mag_lengths[3];
float  	 	state_machine::temp_data[85];
size_t 		state_machine::temp_length;
int32_t 	state_machine::status;



state_machine::state_machine() {
    mA = new Motor(MotorA, true);
    mB = new Motor(MotorB, true);
    servo = new UltraSonicSwivel(S_PULSE, U_PING, 1);
	setup_IMU();
	
    pinMode(M_STDBY, OUTPUT);
    if (!Motor::intTime.begin(Motor::control_interrupt, 1000000 / Motor::freq)) {
        error_string.append("interrupt init fail;");
    }
    stop();
}



state_machine::~state_machine() {
    delete mA;
    delete mB;
    delete servo;
	delete IMU;
}

bool state_machine::setup_IMU() {
	Wire.begin();
	IMU = new MPU9250FIFO(Wire, 0x68);
	status = IMU->begin();	

    snprintf(print_buf, 120, "Status: %d", status);
    Serial.println(print_buf);

    IMU->setAccelRange(MPU9250::AccelRange::ACCEL_RANGE_16G);
    snprintf(print_buf, 120, "Accel Range Success: %d", status);
    Serial.println(print_buf);

    IMU->setGyroRange(MPU9250::GyroRange::GYRO_RANGE_2000DPS);
    snprintf(print_buf, 120, "Gyro Range Success: %d", status);
    Serial.println(print_buf);

    IMU->enableFifo(true, true, true, true);

  	// setting DLPF bandwidth to 20 Hz
  	IMU->setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  	// setting SRD to 19 for a 50 Hz update rate
  	IMU->setSrd(19);
  	// enabling the data ready interrupt
  	IMU->enableDataReadyInterrupt();
  	// attaching the interrupt to microcontroller pin 1
  	pinMode(IMU_INT,INPUT);
  	attachInterrupt(IMU_INT,read_IMU,RISING);

	return status;
}


//Function for initially finding the position of the robot on the course
void state_machine::state_update(){
    //Main loop
    switch(state) { 
    case 0: //Drive towards back wall
        //statements 
        break; 
    case 1: //Drive towards ramp
        //statements
        break; 
    case 2: //Drive up and down ramp
        //statements
        break; 
    case 3: //Identify the post
        //statements
        break; 
    case 4: //Drive towards the post
        //statements
        break; 
    case 5: //Drive towards back wall
        //statements
        break; 
    case 6: //Drive towards ramp
        //statements
        break; 
    case 7: //Drive up and down ramp
        //statements
    case 8: //Drive towards starting location
        //statements
        break; 
}
    
}

void state_machine::ramp_pos_2(){
    //1. Drive towards the ramp 
}

void state_machine::ramp(){
    //1. Drive towards the ramp 
}

void state_machine::pole_id(){
    
}



void state_machine::base_return(){

}

void state_machine::update_speeds() {
    if (speedA > 150) speedA = 150;
    if (speedA < -150) speedA = -150;
    if (speedB > 150) speedB = 150;
    if (speedB < -150) speedB = -150;
    mA->set_speed(speedA);
    mB->set_speed(speedB);
}


void state_machine::read_IMU() {
	status = IMU->readFifo();

    IMU->getFifoAccelX_mss(&accel_lengths[0], accel_data[0]);
    IMU->getFifoAccelY_mss(&accel_lengths[1], accel_data[1]);
    IMU->getFifoAccelZ_mss(&accel_lengths[2], accel_data[2]);
    IMU->getFifoGyroX_rads(&gyro_lengths[0], gyro_data[0]);
    IMU->getFifoGyroY_rads(&gyro_lengths[1], gyro_data[1]);
    IMU->getFifoGyroZ_rads(&gyro_lengths[2], gyro_data[2]);
    IMU->getFifoMagX_uT(&mag_lengths[0], mag_data[0]);
    IMU->getFifoMagY_uT(&mag_lengths[1], mag_data[1]);
    IMU->getFifoMagZ_uT(&mag_lengths[2], mag_data[2]);
    IMU->getFifoTemperature_C(&temp_length, temp_data);

    snprintf(print_buf, 200, "s:%-3d, ax:%-3u, gx:%-3u, mx:%-3u, t:%-3u, AX:%+5f, GX:%+5f, MX:%+5f, T:%+5f", status, accel_lengths[0], gyro_lengths[0],
             mag_lengths[0], temp_length, accel_data[0][0], gyro_data[0][0], mag_data[0][0], temp_data[0]);

}
