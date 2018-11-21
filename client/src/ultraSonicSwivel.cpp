#include "ultraSonicSwivel.h"

UltraSonicSwivel::UltraSonicSwivel(uint8_t servo_pin, uint8_t us_pin, uint32_t millis_per_deg) : servo(), sensor(U_PING, 300) {
    servo.attach(servo_pin);

    servo_set_pos = 90;
    servo.write(servo_set_pos);

    servo_set_timestamp = millis();
    servo_prev_pos = 0;
    //Initialize sensor
    sensor.init();
}

UltraSonicSwivel::~UltraSonicSwivel() {}

bool UltraSonicSwivel::set_position(int32_t new_position) {
    // Constrain the position
    if (new_position > 180) new_position = 180;
    if (new_position < 0) new_position = 0;

    if (get_settled()) {
        this->servo_prev_pos = this->servo_set_pos;
    } else {
        if (new_position > 90)
            this->servo_prev_pos = 180;
        else
            this->servo_prev_pos = 0;
    }

    this->servo_set_timestamp = millis();

    this->servo_set_pos = new_position;

    this->servo.write(this->servo_set_pos);
}

int32_t UltraSonicSwivel::get_position() {
    return this->servo_set_pos;
}

bool UltraSonicSwivel::get_settled() {
    return (millis() - servo_set_timestamp) > abs(servo_set_pos - servo_prev_pos) * millis_per_deg;
}

int32_t UltraSonicSwivel::get_distance() {}

UltraSonicSwivel::swivel_data UltraSonicSwivel::get_reading() {}
