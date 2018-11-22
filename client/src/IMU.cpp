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
IMU* IMU::singleton = nullptr;


IMU::IMU() {
    pitch_compensation = 0;
    yaw_compensation = 0;
    roll_compensation = 0;
    compass_compensation = 0;

    pitch = 0;
    pitch_prev = 0;
}

bool IMU::init() {
    if (singleton != nullptr) return false;
    singleton = this;

    imu = new MPU9250_DMP();

    // Call imu.begin() to verify communication and initialize
    if (imu->begin() != INV_SUCCESS) {
        while (1) {
            Serial.println("Unable to communicate with MPU-9250");
            Serial.println("Check connections, and try again.");
            Serial.println();
            delay(5000);
        }
    }

    imu->dmpBegin(DMP_FEATURE_6X_LP_QUAT |   // Enable 6-axis quat
                      DMP_FEATURE_GYRO_CAL,  // Use gyro calibration
                  FIFO_RATE);                // Set DMP FIFO

    imu->setCompassSampleRate(100);

    // Set up interrupt
    pinMode(IMU_INT, INPUT);
    digitalWrite(IMU_INT, LOW);
    attachInterrupt(IMU_INT, updateIMU, RISING);
    Wire.setClock(400000);
    return true;
}

float IMU::get_pitch() {
    return pitch;
}

Angle IMU::get_pitch_abs() {
    return Angle(imu->pitch + pitch_compensation);
}

Angle IMU::get_yaw_abs() {
    return Angle(imu->yaw + yaw_compensation);
}

Angle IMU::get_roll_abs() {
    return Angle(imu->roll + roll_compensation);
}

Angle IMU::get_compass_abs() {
    return Angle(imu->computeCompassHeading() + compass_compensation);
}

void IMU::compensate_pitch(float coefficient, Angle angle) {
    if (coefficient < 0) return;
    if (coefficient > 1) coefficient = 1;
    pitch_compensation -= angle.distance(get_pitch_abs()) * coefficient;
}

void IMU::compensate_yaw(float coefficient, Angle angle) {
    if (coefficient < 0) return;
    if (coefficient > 1) coefficient = 1;
    yaw_compensation -= angle.distance(get_yaw_abs()) * coefficient;
    compass_compensation -= angle.distance(get_yaw_abs()) * coefficient;
}

void IMU::compensate_roll(float coefficient, Angle angle) {
    if (coefficient < 0) return;
    if (coefficient > 1) coefficient = 1;
    roll_compensation -= angle.distance(get_roll_abs()) * coefficient;
}

void IMU::compensate_compass(float coefficient, Angle angle) {
    if (coefficient < 0) return;
    if (coefficient > 1) coefficient = 1;
    compass_compensation -= angle.distance(get_compass_abs()) * coefficient;
}


void IMU::stabilize() {
    Angle prev_yaw;
    pinMode(GRN_LED, OUTPUT);
    digitalWrite(GRN_LED, 0);  // Make sure LED off

    // Loop while it is drifting
    do {
        prev_yaw = get_yaw_abs();
        Serial.println(prev_yaw.as_float());
        delayMicroseconds(4000000);  // delay two seconds
    } while (abs(prev_yaw.distance(get_yaw_abs())) > 0.05);
    Serial.println(get_yaw_abs().as_float());

    compensate_yaw(1, Angle(0));
    compensate_compass(1, Angle(0));

    digitalWrite(GRN_LED, 1);  // signal value is good
}

void IMU::complementary_compass_filter() {
    yaw_compensation -= get_compass_abs().distance(get_yaw_abs()) * 0.01;
}


void IMU::updateIMU() {
    static bool LED_state = false;
    digitalWrite(13, LED_state);
    LED_state = !LED_state;
    // Check for new data in the FIFO
    if (imu->fifoAvailable()) {
        // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
        if (imu->dmpUpdateFifo() == INV_SUCCESS) {
            // computeEulerAngles can be used -- after updating the
            // quaternion values -- to estimate roll, pitch, and yaw
            imu->computeEulerAngles();
            imu->updateCompass();

            singleton->complementary_compass_filter();

            if ((imu->pitch - singleton->pitch_prev) < -20) {
                singleton->pitch += imu->pitch - singleton->pitch_prev + 360;
            } else if (imu->pitch - singleton->pitch_prev > 20) {
                singleton->pitch += imu->pitch - singleton->pitch_prev - 360;
            } else {
                singleton->pitch += imu->pitch - singleton->pitch_prev;
            }

            singleton->pitch_prev = imu->pitch;
        }
    }
}
