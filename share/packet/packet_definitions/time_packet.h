/*******************************************************************************
 *
 * FILENAME: time_packet.h
 *
 * PROJECT: RampBotHost
 *
 * ORIGINAL AUTHOR: Lyle Cheatham
 *
 * DATE: 10/3/18
 *
 * COPYRIGHT THE ORIGINAL AUTHORS, ALL RIGHTS RESERVED UNLESS OTHERWISE NOTED
 *
 *******************************************************************************/

#pragma once

#include "packet_data.h"

class Time_Packet : public Packet_Data {
public:
    Time_Packet(uint32_t time_stamp, bool set_time, bool request_time);
    Time_Packet();

    static inline Packet_Data* create_packet() { return new Time_Packet(); }


    packet_error deserialize(const char* data, uint8_t size) override;
    std::pair<packet_error, std::vector<char>> serialize() override;
    packet_id get_packet_id() override;

    uint32_t get_time();

private:
    uint32_t time_stamp;
    bool set_time;
    bool request_time;
};
