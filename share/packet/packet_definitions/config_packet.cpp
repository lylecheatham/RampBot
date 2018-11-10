/*******************************************************************************
 *
 * FILENAME: config_packet.cpp
 *
 * PROJECT: RampBotHost
 *
 * ORIGINAL AUTHOR: Lyle Cheatham
 *
 * DATE: 10/14/18
 *
 * COPYRIGHT THE ORIGINAL AUTHORS, ALL RIGHTS RESERVED UNLESS OTHERWISE NOTED
 *
 *******************************************************************************/

#include "config_packet.h"
#include "packet_utils.h"


Config_Packet::Config_Packet() : message(heartbeat) {}

Config_Packet::Config_Packet(message_type message) : message(message) {}

packet_error Config_Packet::deserialize(const char *data, uint8_t size) {
    if (size > 1) return p_err_oversize;
    if (size < 1) return p_err_undersize;

    packet_utils::unpack(data, 0, message);

    return p_err_none;
}

std::pair<packet_error, std::vector<char>> Config_Packet::serialize() {
    std::vector<char> data;
    packet_utils::pack(data, message);

    return std::make_pair(p_err_none, data);
}

packet_id Config_Packet::get_packet_id() {
    return packet_id::config_packet;
}

void Config_Packet::set_message(message_type message) {
    this->message = message;
}

Config_Packet::message_type Config_Packet::get_message() {
    return this->message;
}
