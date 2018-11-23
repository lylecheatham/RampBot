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

    pitch_lp = 0;
    yaw_lp = 0;
    roll_lp = 0;

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
    imu->begin();

    imu->setCompassSampleRate(100);

    // Set up interrupt
    pinMode(IMU_INT, INPUT);
    digitalWrite(IMU_INT, LOW);
    attachInterrupt(IMU_INT, updateIMU, RISING);
    Wire.setClock(400000);
    return true;
}

float IMU::get_accel_y() {
	return imu->calcAccel(imu->ay);
}

float IMU::get_pitch() const {
    return pitch;
}

Angle IMU::get_pitch_abs() const {
    return Angle(imu->pitch + pitch_compensation);
}

Angle IMU::get_yaw_abs() const {
    return Angle(imu->yaw + yaw_compensation);
}

Angle IMU::get_roll_abs() const {
    return Angle(imu->roll + roll_compensation);
}

Angle IMU::get_compass_abs() const {
    return Angle(imu->computeCompassHeading() + compass_compensation);
}

void IMU::compensate_pitch(const float coefficient, const Angle angle) {
    pitch_compensation += compensate_float(get_pitch_abs(), coefficient, angle);
}

void IMU::compensate_yaw(const float coefficient, const Angle angle) {
    yaw_compensation += compensate_float(get_yaw_abs(), coefficient, angle);
    compass_compensation += compensate_float(get_yaw_abs(), coefficient, angle);
}

void IMU::compensate_roll(const float coefficient, const Angle angle) {
    roll_compensation += compensate_float(get_roll_abs(), coefficient, angle);
}

void IMU::compensate_compass(const float coefficient, const Angle angle) {
    compass_compensation += compensate_float(get_compass_abs(), coefficient, angle);
}

void IMU::complement(Angle& to_complement, float coefficient, const Angle complement_with) {
    if (coefficient < 0) return;
    if (coefficient > 1) coefficient = 1;
    to_complement += to_complement.distance(complement_with) * coefficient;
}

float IMU::compensate_float(const Angle to_compensate, float coefficient, const Angle compensate_with) {
    if (coefficient < 0) return 0;
    if (coefficient > 1) coefficient = 1;
    return to_compensate.distance(compensate_with) * coefficient;
}


Angle IMU::get_pitch_lp() const {
    return pitch_lp + pitch_compensation;
}

Angle IMU::get_yaw_lp() const {
    return yaw_lp + yaw_compensation;
}

Angle IMU::get_roll_lp() const {
    return roll_lp + roll_compensation;
}

void IMU::stabilize() {
    Angle prev_yaw;
    pinMode(GRN_LED, OUTPUT);
    digitalWrite(GRN_LED, 0);  // Make sure LED off
    char buf[200];

    // Loop while it is drifting
    do {
        prev_yaw = get_yaw_abs();
        snprintf(buf, 200, "Calibrating... Yaw value: %f", prev_yaw.as_float());
        Serial.println(buf);
        delay(8000);  // delay eight seconds
    } while (abs(prev_yaw.distance(get_yaw_abs())) > 0.05);
    snprintf(buf, 200, "Done!    Final Yaw value: %f", get_yaw_abs().as_float());
    Serial.println(buf);

    compensate_yaw(1, Angle(0));
    compensate_compass(1, Angle(0));
    compensate_pitch(1, Angle(0));
    compensate_roll(1, Angle(0));

    snprintf(buf,
             200,
             "yaw: %f\ncompass: %f\npitch: %f\nroll: %f\n",
             get_yaw_abs().as_float(),
             get_compass_abs().as_float(),
             get_pitch_abs().as_float(),
             get_roll_abs().as_float());
    Serial.println(buf);

    digitalWrite(GRN_LED, 1);  // signal value is good
}

void IMU::complementary_compass_filter() {
    yaw_compensation += get_yaw_abs().distance(get_compass_abs()) * 0.01;
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
            imu->updateAccel();

            complement(singleton->pitch_lp, pitch_lp_constant, Angle(imu->pitch));
            complement(singleton->yaw_lp, yaw_lp_constant, Angle(imu->yaw));
            complement(singleton->roll_lp, roll_lp_constant, Angle(imu->roll));

            //singleton->complementary_compass_filter();

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
