/* main.cpp
 * MTE 380 Design Project
 * Original Author(s): Lyle Cheatham / Eric Murphy-Zaremba / Conner Currie
 * Creation Date: Oct 20 /2018
 */


#include <Arduino.h>
#include <MPU9250.h>
#include <PWMServo.h>
#include <NewPing.h>

#include "packet.h"
#include "sensor_packet.h"
#include "constants.h"
#include "DemoInterface.hpp"


// Hacky stuff, don't remove
extern "C" {
int _getpid() { return -1; }
int _kill(int pid, int sig) { return -1; }
int _write() { return -1; }
}

DemoInterface* demo;

void setup(){

	analogWriteResolution(12);
	
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
	/*
	std::unique_ptr<Packet_Data> data(new Sensor_Packet());
	Packet test (node_id::rasp_pi, packet_id::sensor_packet, std::move(data));
    auto result = test.serialize();
    if(result.first == p_err_none){
        Serial.write(result.second.data(), result.second.size());
    }*/

	// Startup the demo
	demo = new DemoInterface();
	Serial.println("Starting...");

    Wire.begin();

    // The IMU
    MPU9250FIFO IMU(Wire, 0x68);

    char print_buf[200];

    int status = IMU.begin();
    snprintf(print_buf, 120, "Status: %d", status);
    Serial.println(print_buf);

    IMU.setAccelRange(MPU9250::AccelRange::ACCEL_RANGE_16G);
    snprintf(print_buf, 120, "Accel Range Success: %d", status);
    Serial.println(print_buf);

    IMU.setGyroRange(MPU9250::GyroRange::GYRO_RANGE_2000DPS);
    snprintf(print_buf, 120, "Gyro Range Success: %d", status);
    Serial.println(print_buf);

    IMU.enableFifo(true, true, true, true);

    // Loop goes here
    while(1){
        if(false){/*(status != -1){*/
            float accel_data[3][85];
            size_t accel_lengths[3];
            float gyro_data[3][85];
            size_t gyro_lengths[3];
            float mag_data[3][85];
            size_t mag_lengths[3];
            float temp_data[85];
            size_t temp_length;

            int status = IMU.readFifo();

            IMU.getFifoAccelX_mss(&accel_lengths[0], accel_data[0]);
            IMU.getFifoAccelY_mss(&accel_lengths[1], accel_data[1]);
            IMU.getFifoAccelZ_mss(&accel_lengths[2], accel_data[2]);
            IMU.getFifoGyroX_rads(&gyro_lengths[0], gyro_data[0]);
            IMU.getFifoGyroY_rads(&gyro_lengths[1], gyro_data[1]);
            IMU.getFifoGyroZ_rads(&gyro_lengths[2], gyro_data[2]);
            IMU.getFifoMagX_uT(&mag_lengths[0], mag_data[0]);
            IMU.getFifoMagY_uT(&mag_lengths[1], mag_data[1]);
            IMU.getFifoMagZ_uT(&mag_lengths[2], mag_data[2]);
            IMU.getFifoTemperature_C(&temp_length, temp_data);

            snprintf(print_buf, 200, "s:%-3d, ax:%-3u, gx:%-3u, mx:%-3u, t:%-3u, AX:%+5f, GX:%+5f, MX:%+5f, T:%+5f",
                    status,
                    accel_lengths[0],
                    gyro_lengths[0],
                    mag_lengths[0],
                    temp_length,
                    accel_data[0][0],
                    gyro_data[0][0],
                    mag_data[0][0],
                    temp_data[0]
                    );
            /* Serial.println(print_buf); */
        }

        float read_voltage_A = ((float)analogRead(A8) * 33 / 10) / 65535;
        float read_voltage_B = ((float)analogRead(A9) * 33 / 10) / 65535;

        snprintf(print_buf, 200, "A:%+5f, B:%+5f", read_voltage_A, read_voltage_B);

        Serial.println(print_buf);

        int8 rcv_char = DemoInterface::get_char();
        demo->run_command(rcv_char);
    }
}

void loop(){}
