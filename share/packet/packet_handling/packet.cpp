/*******************************************************************************
 *
 * FILENAME: packet.cpp
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

#include "packet.h"
#include <vector>
#include "packet_externs.h"
#include "packet_utils.h"

// This library is incompatible otherwise
static_assert(std::is_same<std::uint8_t, unsigned char>::value,
              "We require std::uint8_t to be implemented as unsigned char");

Packet::Packet() : header(null_node, null_packet), data(nullptr) {}

Packet::Packet(node_id destination, packet_id type, std::unique_ptr<Packet_Data> data_ptr) : header(destination, type),
                                                                                   data(std::move(data_ptr)) {}
Packet::~Packet(){}

std::pair<packet_error, std::vector<char>> Packet::serialize() {
    std::vector<char> data_bytes;

    // if there is data
    if (data != nullptr) {
        // Start by serializing the data so we know the length
        auto data_serialized = data->serialize();

        // Check for error
        if (data_serialized.first != p_err_none) return data_serialized;

        // Get the data bytes
        data_bytes = data_serialized.second;
    } else {
        // empty vector otherwise
        data_bytes = std::vector<char>();
    }

    // check to make sure data length fits in a uint8_t
    if (data_bytes.size() > UINT8_MAX) return std::make_pair(p_err_oversize, std::vector<char>());

    // Set the length in the header
    this->header.set_length((uint8_t) data_bytes.size());

    // Serialize the header
    std::vector<char> packet_bytes = this->header.serialize();

    // Attach the data to the header
    packet_bytes.insert(packet_bytes.end(), data_bytes.begin(), data_bytes.end());

    // Calculate CRC for entire packet
    tail_crc = packet_externs::packet_get_crc(packet_bytes);

    // Append the CRC
    packet_utils::pack(packet_bytes, tail_crc);

    return std::make_pair(p_err_none, packet_bytes);
}


packet_error Packet::deserialize(const char *data_bytes) {
    // deserialize the header
    packet_error err = this->header.deserialize(data_bytes);
    if (err != p_err_none) return err;

    // now that we know the length, find where the crc on the end is and check it against the data_bytes
    uint16_t calculated_crc = packet_externs::packet_get_crc(data_bytes, Packet_Header::header_length + header.get_length());

    packet_utils::unpack(data_bytes, Packet_Header::header_length + header.get_length(), tail_crc);

    // compare the CRCs
    if (tail_crc != calculated_crc) return p_err_bad_crc;

    data = std::move(packet_externs::packet_from_id(header.get_packet_id()));

    if(data != nullptr) {
        // Deserialize the data
        data->deserialize(data_bytes + Packet_Header::header_length, header.get_length());
    } else {
        if(header.get_length() != 0){
            return p_err_oversize;
        }
    }

    return p_err_none;
}

std::unique_ptr<Packet_Data> Packet::reacquire_data(){
    return std::move(data);
}


uint16_t Packet::get_serial_number() const{
    return header.get_serial_number();
}
