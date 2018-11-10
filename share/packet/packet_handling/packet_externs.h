/*******************************************************************************
 *
 * FILENAME: packet_externs.h
 *
 * PROJECT: RampBotHost
 *
 * ORIGINAL AUTHOR: Lyle Cheatham
 *
 * DATE: 10/15/18
 *
 * COPYRIGHT THE ORIGINAL AUTHORS, ALL RIGHTS RESERVED UNLESS OTHERWISE NOTED
 *
 *******************************************************************************/

#pragma once
#include <cstddef>
#include <memory>
#include <vector>
#include "packet_defs.h"

class Packet_Data;

namespace packet_externs {

uint16_t get_serial_packet(node_id destination);

// CRC needs to be CRC-16
uint16_t packet_get_crc(std::vector<char> data);

// CRC needs to be CRC-16
uint16_t packet_get_crc(const char *data, size_t length);

extern node_id this_node;

std::unique_ptr<Packet_Data> packet_from_id(packet_id id);

}  // namespace packet_externs
