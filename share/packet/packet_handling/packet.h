/*******************************************************************************
 *
 * FILENAME: packet.h
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

#include <memory>
#include <tuple>
#include <vector>
#include "packet_data.h"
#include "packet_defs.h"
#include "packet_header.h"


class Packet {
public:
    Packet();
    Packet(node_id destination, packet_id type, std::unique_ptr<Packet_Data> data_ptr = nullptr);
    ~Packet();

    std::pair<packet_error, std::vector<char>> serialize();

    packet_error deserialize(const char *data_bytes);

    std::unique_ptr<Packet_Data> reacquire_data();

    uint16_t get_serial_number() const;

private:
    Packet_Header header;
    std::unique_ptr<Packet_Data> data;
    uint16_t tail_crc;
};
