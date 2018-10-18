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

class Packet_Data;
class Packet;

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

    typedef struct{
        std::unique_ptr<Packet> packet;
        uint16_t serial_number;
        void (*callback_void)();
        void (*callback_data)(std::unique_ptr<Packet_Data>);
    } send_entry;

    boost::circular_buffer<send_entry> sent_list;

    BufferedAsyncSerial serial;

};


