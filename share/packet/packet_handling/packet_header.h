/*******************************************************************************
 *
 * FILENAME: packet_header.h
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

#include "packet_defs.h"
#include "packet_errors.h"
#include <vector>
#include <tuple>

class Packet_Header {
public:
    static constexpr size_t header_length = 9;

    Packet_Header();

    Packet_Header(node_id destination, packet_id type, uint8_t length = 0);

    void set_length(uint8_t length);
    uint8_t get_length();

    packet_id get_packet_id();

    std::vector<char> serialize();
    packet_error deserialize(const char *data);

    uint16_t get_serial_number() const;


private:
    // All members are in packing order
    uint8_t length;
    node_id destination;
    packet_id packet_type;
    node_id source;

    uint16_t source_dest_serial_number;
    uint16_t header_crc;
};
