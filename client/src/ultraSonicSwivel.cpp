#include "ultraSonicSwivel.h"

UltraSonicSwivel::UltraSonicSwivel(uint8_t servo_pin, uint8_t us_pin, uint32_t millis_per_deg) : servo(), sensor(us_pin, us_pin, 200) {
    servo.attach(servo_pin);

    servo_set_pos = 90;
    servo.write(servo_set_pos);

    servo_set_timestamp = millis();
    servo_prev_pos = 0;

}

UltraSonicSwivel::~UltraSonicSwivel() {

}

int32_t UltraSonicSwivel::set_position(){

}

int32_t UltraSonicSwivel::get_position(){

}

bool UltraSonicSwivel::get_settled(){
    return (millis() - servo_set_timestamp) >
        abs(servo_set_pos - servo_prev_pos) * millis_per_deg;

}

int32_t UltraSonicSwivel::get_distance(){

}

UltraSonicSwivel::swivel_data UltraSonicSwivel::get_reading(){

}

