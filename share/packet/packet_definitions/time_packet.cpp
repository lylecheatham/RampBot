/*******************************************************************************
 *
 * FILENAME: time_packet.cpp
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

#include "time_packet.h"
#include <bitset>
#include "packet_utils.h"

Time_Packet::Time_Packet(uint32_t time_stamp, bool set_time, bool request_time) {
    this->time_stamp = time_stamp;
    this->set_time = set_time;
    this->request_time = request_time;
}

Time_Packet::Time_Packet() {
    this->time_stamp = 0;
    this->set_time = false;
    this->request_time = false;
}

packet_error Time_Packet::deserialize(const char *data, uint8_t size) {
    if (size < 5) return p_err_undersize;
    if (size > 5) return p_err_oversize;

    packet_utils::unpack(data, 0, time_stamp);

    std::bitset<2> flags(&data[4]);

    set_time = flags[0];
    request_time = flags[1];

    return p_err_none;
}

std::pair<packet_error, std::vector<char>> Time_Packet::serialize() {
    std::vector<char> data;

    packet_utils::pack(data, time_stamp);

    std::bitset<2> flags;
    flags[0] = set_time;
    flags[1] = request_time;

    data.push_back((uint8_t)flags.to_ulong());

    return std::make_pair(p_err_none, data);
}

packet_id Time_Packet::get_packet_id() {
    return packet_id::time_packet;
}
