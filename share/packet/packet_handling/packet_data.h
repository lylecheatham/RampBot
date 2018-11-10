/*******************************************************************************
 *
 * FILENAME: packet_Data.h
 *
 * PROJECT: RampBotHost
 *
 * ORIGINAL AUTHOR: Lyle Cheatham
 *
 * DATE: 10/4/18
 *
 * COPYRIGHT THE ORIGINAL AUTHORS, ALL RIGHTS RESERVED UNLESS OTHERWISE NOTED
 *
 *******************************************************************************/

#pragma once
#include <vector>
#include "packet_defs.h"
#include "packet_errors.h"


class Packet_Data {
public:
    virtual ~Packet_Data(){};

    virtual std::pair<packet_error, std::vector<char>> serialize() = 0;
    virtual packet_error deserialize(const char *data, uint8_t size) = 0;
    virtual packet_id get_packet_id() = 0;

protected:
    Packet_Data(){};
};
