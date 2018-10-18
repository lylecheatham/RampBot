/*******************************************************************************
 *
 * FILENAME: packet_header.cpp
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

#include "packet_header.h"
#include "packet_utils.h"
#include "packet_externs.h"

Packet_Header::Packet_Header(){
    this->source = null_node;
    this->destination = null_node;
    this->source_dest_serial_number = 0;
    this->length = 0;
    this->packet_type = null_packet;
}

Packet_Header::Packet_Header(node_id destination, packet_id type, uint8_t length){
    this->source = packet_externs::this_node;
    this->destination = destination;
    this->source_dest_serial_number = packet_externs::get_serial_packet(destination);
    this->length = length;
    this->packet_type = type;
};

void Packet_Header::set_length(uint8_t length){
    this->length = length;
}

uint8_t Packet_Header::get_length(){
    return this->length;
};

packet_id Packet_Header::get_packet_id(){
    return this->packet_type;
}

std::vector<char> Packet_Header::serialize() {
    std::vector<char> data;

    // First we add our catch character, which is an asterisk because oit looks like 01010100 in ascii
    // The catch character is useful if we get out of sync
    data.push_back('*');

    // Length is the most useful because it tells us how long the packet is
    // We can quickly check packet validity by checking the header crc and then seeing if the data crc is right
    packet_utils::pack(data, length);

    // Destination is the next most useful, do a quick check to see if it's for us or not
    packet_utils::pack(data, destination);

    // Packet type is the next most useful piece of information, if it's for us we know what to do with it
    packet_utils::pack(data, packet_type);

    // Source is useful for sending the ack
    packet_utils::pack(data, source);

    // Useful for the ack
    packet_utils::pack(data, source_dest_serial_number);

    // Calculate the crc
    header_crc = packet_externs::packet_get_crc(data);

    // Append the crc
    packet_utils::pack(data, header_crc);

    return data;
}

packet_error Packet_Header::deserialize(const char *data){
    // Check for the catch character
    if(data[0] != '*') return p_err_no_catch_character;

    // Calculate the CRC
    uint16_t calculated_crc = packet_externs::packet_get_crc(data, 7);

    // Check the CRC on the end for a match
    packet_utils::unpack(data, 7, header_crc);
    if(header_crc != calculated_crc) return p_err_bad_crc;

    // Unpack the rest
    size_t index = 1;
    packet_utils::unpack_increment(data, index, length);
    packet_utils::unpack_increment(data, index, destination);
    packet_utils::unpack_increment(data, index, packet_type);
    packet_utils::unpack_increment(data, index, source);
    packet_utils::unpack_increment(data, index, source_dest_serial_number);

    return p_err_none;
}

uint16_t Packet_Header::get_serial_number() const {
    return source_dest_serial_number;
}
