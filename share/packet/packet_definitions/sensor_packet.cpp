/*******************************************************************************
 *
 * FILENAME: sensor_packet.cpp
 *
 * PROJECT: RampBotHost
 *                    
 * ORIGINAL AUTHOR: Lyle Cheatham                       
 *
 * DATE: 10/6/18
 *
 * COPYRIGHT THE ORIGINAL AUTHORS, ALL RIGHTS RESERVED UNLESS OTHERWISE NOTED
 *
 *******************************************************************************/

#include "sensor_packet.h"
#include "packet_utils.h"
#include <bitset>


Sensor_Packet::Sensor_Packet() {
    ultrasonic_reading.first = false;
    encoder_reading.first = false;
    gyroscope_reading.first = false;
    accelerometer_reading.first = false;
    compass_reading.first = false;
}

packet_error Sensor_Packet::deserialize(const char *data, uint8_t size) {
    // The timestamp and flags are 5 long, undersize if not at least that.
    uint8_t expected_size = 5;
    if (size < expected_size) return p_err_undersize;

    // Unpack the timestamp
    packet_utils::unpack(data, 0, timestamp);

    // Unpack the flags
    std::bitset<5> flags(&data[4]);
    ultrasonic_reading.first = flags[0];
    encoder_reading.first = flags[1];
    gyroscope_reading.first = flags[2];
    accelerometer_reading.first = flags[3];
    compass_reading.first = flags[4];


    // Calculate the expected length based on flags
    if (ultrasonic_reading.first){
        expected_size += 4;
    }
    if (encoder_reading.first){
        expected_size += 8;
    }
    if (gyroscope_reading.first){
        expected_size += 12;
    }
    if (accelerometer_reading.first){
        expected_size += 12;
    }
    if (compass_reading.first){
        expected_size += 12;
    }

    // Error if the size is wrong
    if (size > expected_size) return p_err_oversize;
    if (size < expected_size) return p_err_undersize;

    size_t index = 5;

    // Start unpacking the data based on the flags
    if (ultrasonic_reading.first) {
        packet_utils::unpack_increment(data, index, ultrasonic_reading.second);
    }

    if (encoder_reading.first) {
        packet_utils::unpack_increment(data, index, encoder_reading.second[0]);
        packet_utils::unpack_increment(data, index, encoder_reading.second[1]);
    }

    if (gyroscope_reading.first) {
        packet_utils::unpack_increment(data, index, gyroscope_reading.second[0]);
        packet_utils::unpack_increment(data, index, gyroscope_reading.second[1]);
        packet_utils::unpack_increment(data, index, gyroscope_reading.second[3]);
    }

    if (accelerometer_reading.first) {
        packet_utils::unpack_increment(data, index, accelerometer_reading.second[0]);
        packet_utils::unpack_increment(data, index, accelerometer_reading.second[1]);
        packet_utils::unpack_increment(data, index, accelerometer_reading.second[3]);
    }

    if (compass_reading.first) {
        packet_utils::unpack_increment(data, index, compass_reading.second[0]);
        packet_utils::unpack_increment(data, index, compass_reading.second[1]);
        packet_utils::unpack_increment(data, index, compass_reading.second[3]);
    }
    return p_err_none;
}

std::pair<packet_error, std::vector<char>> Sensor_Packet::serialize() {
    std::vector<char> data;

    packet_utils::pack(data, timestamp);

    std::bitset<5> flags;

    flags[0] = ultrasonic_reading.first;
    flags[1] = encoder_reading.first;
    flags[2] = gyroscope_reading.first;
    flags[3] = accelerometer_reading.first;
    flags[4] = compass_reading.first;

    data.push_back((char) flags.to_ulong());

    if (ultrasonic_reading.first) {
        packet_utils::pack(data, ultrasonic_reading.second);
    }

    if (encoder_reading.first) {
        packet_utils::pack(data, encoder_reading.second[0]);
        packet_utils::pack(data, encoder_reading.second[1]);
    }

    if (gyroscope_reading.first) {
        packet_utils::pack(data, gyroscope_reading.second[0]);
        packet_utils::pack(data, gyroscope_reading.second[1]);
        packet_utils::pack(data, gyroscope_reading.second[3]);
    }

    if (accelerometer_reading.first) {
        packet_utils::pack(data, accelerometer_reading.second[0]);
        packet_utils::pack(data, accelerometer_reading.second[1]);
        packet_utils::pack(data, accelerometer_reading.second[3]);
    }

    if (compass_reading.first) {
        packet_utils::pack(data, compass_reading.second[0]);
        packet_utils::pack(data, compass_reading.second[1]);
        packet_utils::pack(data, compass_reading.second[3]);
    }

    return std::make_pair(p_err_none, data);
}

packet_id Sensor_Packet::get_packet_id(){
    return packet_id::sensor_packet;
}

void Sensor_Packet::set_timestamp(uint32_t timestamp) {
    this->timestamp = timestamp;
}

uint32_t Sensor_Packet::get_timestamp() const {
    return this->timestamp;
}

void Sensor_Packet::set_ultrasonic_reading(int32_t ultrasonic_reading) {
    this->ultrasonic_reading.first = true;
    this->ultrasonic_reading.second = ultrasonic_reading;
}

bool Sensor_Packet::has_ultrasonic_reading() const {
    return this->ultrasonic_reading.first;
}

int32_t Sensor_Packet::get_ultrasonic_reading() const {
    return this->ultrasonic_reading.second;
}

void Sensor_Packet::set_encoder_reading(std::array<int32_t, 2> encoder_reading) {
    this->encoder_reading.first = true;
    this->encoder_reading.second = encoder_reading;
}

bool Sensor_Packet::has_encoder_reading() const {
    return this->encoder_reading.first;
}

std::array<int32_t, 2> Sensor_Packet::get_encoder_reading() const {
    return this->encoder_reading.second;
}

void Sensor_Packet::set_gyroscope_reading(std::array<float, 3> gyroscope_reading) {

    this->gyroscope_reading.first = true;
    this->gyroscope_reading.second = gyroscope_reading;
}

bool Sensor_Packet::has_gyroscope_reading() const {
    return this->gyroscope_reading.first;

}

std::array<float, 3> Sensor_Packet::get_gyroscope_reading() const {
    return this->gyroscope_reading.second;
}

void Sensor_Packet::set_accelerometer_reading(std::array<float, 3> accelerometer_reading) {
    this->accelerometer_reading.first = true;
    this->accelerometer_reading.second = accelerometer_reading;

}

bool Sensor_Packet::has_accelerometer_reading() const {
    return this->accelerometer_reading.first;

}

std::array<float, 3> Sensor_Packet::get_accelerometer_reading() const {
    return this->accelerometer_reading.second;

}

void Sensor_Packet::set_compass_reading(std::array<float, 3> compass_reading) {
    this->compass_reading.first = true;
    this->compass_reading.second = compass_reading;
}

bool Sensor_Packet::has_compass_reading() const {
    return this->compass_reading.first;
}

std::array<float, 3> Sensor_Packet::get_compass_reading() const {
    return this->compass_reading.second;
}
