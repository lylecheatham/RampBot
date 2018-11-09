/*******************************************************************************
 *
 * FILENAME: PacketHandler.h
 *
 * PROJECT: RampBotHost
 *                    
 * ORIGINAL AUTHOR: Lyle Cheatham                       
 *
 * DATE: 10/16/18
 *
 * COPYRIGHT THE ORIGINAL AUTHORS, ALL RIGHTS RESERVED UNLESS OTHERWISE NOTED
 *
 *******************************************************************************/

#pragma once
#include <boost/circular_buffer.hpp>
#include "BufferedAsyncSerial.h"
#include "packet_data.h"
#include "packet.h"



class SendEntry {
public:
    SendEntry(std::unique_ptr<Packet> _packet,
              uint16_t _serial_number) :
        packet(std::move(_packet)),
        serial_number(_serial_number),
        callback_void(nullptr),
        callback_data(nullptr) {}

    SendEntry(std::unique_ptr<Packet> _packet,
              uint16_t _serial_number,
              void (*_callback_data)(std::unique_ptr<Packet_Data>)) :
        packet(std::move(_packet)),
        serial_number(_serial_number),
        callback_void(nullptr),
        callback_data(_callback_data) {}

    SendEntry(std::unique_ptr<Packet> _packet,
              uint16_t _serial_number,
              void (*_callback_void)()) :
        packet(std::move(_packet)),
        serial_number(_serial_number),
        callback_void(_callback_void),
        callback_data(nullptr) {}

private:
    std::unique_ptr<Packet> packet;
    uint16_t serial_number;
    void (*callback_void)();
    void (*callback_data)(std::unique_ptr<Packet_Data>);
};

class PacketHandler {
public:
    PacketHandler();

    bool send_packet(std::unique_ptr<Packet_Data> data);
    bool send_packet(std::unique_ptr<Packet_Data> data, void (*callback)());
    bool send_packet(std::unique_ptr<Packet_Data> data, void (*callback)(std::unique_ptr<Packet_Data>));

    bool run_receive_packet();


private:
    std::pair<bool, std::unique_ptr<Packet> > send_packet_impl(std::unique_ptr<Packet_Data> data);
    void port_error_handler();

    boost::circular_buffer<SendEntry> sent_list;

    BufferedAsyncSerial serial;

};


