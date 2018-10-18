/*******************************************************************************
 *
 * FILENAME: sensor_packet.h
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

#pragma once

#include "packet_data.h"
#include <array>


class Sensor_Packet : public Packet_Data {
public:
    Sensor_Packet();

    static inline Packet_Data *create_packet() {
        return new Sensor_Packet();
    }

    packet_error deserialize(const char *data, uint8_t size) override;
    std::pair<packet_error, std::vector<char> > serialize() override;
    packet_id get_packet_id() override;

    void set_timestamp(uint32_t timestamp);
    uint32_t get_timestamp() const;

    void set_ultrasonic_reading(int32_t ultrasonic_reading);
    bool has_ultrasonic_reading() const;
    int32_t get_ultrasonic_reading() const;

    void set_encoder_reading(std::array<int32_t, 2> encoder_reading);
    bool has_encoder_reading() const;
    std::array<int32_t, 2> get_encoder_reading() const;

    void set_gyroscope_reading(std::array<float, 3> gyroscope_reading);
    bool has_gyroscope_reading() const;
    std::array<float, 3> get_gyroscope_reading() const;

    void set_accelerometer_reading(std::array<float, 3> accelerometer_reading);
    bool has_accelerometer_reading() const;
    std::array<float, 3> get_accelerometer_reading() const;

    void set_compass_reading(std::array<float, 3> compass_reading);
    bool has_compass_reading() const;
    std::array<float, 3> get_compass_reading() const;

private:
    uint32_t timestamp;

    std::pair<bool, int32_t> ultrasonic_reading;

    std::pair<bool, std::array<int32_t, 2> > encoder_reading;

    std::pair<bool, std::array<float, 3> > gyroscope_reading;

    std::pair<bool, std::array<float, 3> > accelerometer_reading;

    std::pair<bool, std::array<float, 3> > compass_reading;
};

