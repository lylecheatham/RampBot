/*******************************************************************************
 *
 * FILENAME: PacketHandler.cpp
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


#include "PacketHandler.h"
#include "packet_data.h"
#include "packet.h"
#include "packet_externs.h"

#include "config_packet.h"
#include "time_packet.h"
#include "sensor_packet.h"


struct packet_creation_entry {
    packet_id id;
    Packet_Data *(*func)();
};


static std::array<packet_creation_entry, number_packet_ids> packet_creation_list =
    {{
         {.id=null_packet, .func=nullptr},
         {.id=packet_response, .func=nullptr},
         {.id=config_packet, .func=Config_Packet::create_packet},
         {.id=time_packet, .func=Time_Packet::create_packet},
         {.id=sensor_packet, .func=Sensor_Packet::create_packet},
         {.id=motor_packet, .func=nullptr},
     }};


std::unique_ptr<Packet_Data> packet_externs::packet_from_id(packet_id id) {
    for (auto entry : packet_creation_list) {
        if (id == entry.id) {
            return std::unique_ptr<Packet_Data>(entry.func());
        }
    }
    return nullptr;
}

PacketHandler::PacketHandler() :
    sent_list(50),
    serial() {
    serial.open("/dev/cu.usbserial-00000000", 115200);

    send_packet(std::unique_ptr<Packet_Data>(new Config_Packet(Config_Packet::wakeup)));

}

bool PacketHandler::send_packet(std::unique_ptr<Packet_Data> data) {
    auto response = send_packet_impl(std::move(data));

    if(!response.first) return false;

    // TODO: Fix this


    sent_list.push_back(SendEntry(std::move(response.second), response.second->get_serial_number()));

    return true;
}

bool PacketHandler::send_packet(std::unique_ptr<Packet_Data> data, void (*callback)()) {
    auto response = send_packet_impl(std::move(data));

    if(!response.first) return false;

    sent_list.push_back(SendEntry(std::move(response.second), response.second->get_serial_number(), callback));

    return true;
}

bool PacketHandler::send_packet(std::unique_ptr<Packet_Data> data, void (*callback)(std::unique_ptr<Packet_Data>)) {
    auto response = send_packet_impl(std::move(data));

    if(!response.first) return false;

    sent_list.push_back(SendEntry(std::move(response.second), response.second->get_serial_number(), callback));

    return true;
}

bool PacketHandler::run_receive_packet() {
    return false;
}

std::pair<bool, std::unique_ptr<Packet> > PacketHandler::send_packet_impl(std::unique_ptr<Packet_Data> data){
    // Create the packet
    std::unique_ptr<Packet> packet(new Packet(node_id::micro, data->get_packet_id(), std::move(data)));

    // Serialize it
    auto serialized = packet->serialize();

    // TODO: do something with the packet here
    if (serialized.first != p_err_none) return std::make_pair(false, std::move(packet));

    serial.write(serialized.second);

    if (serial.errorStatus()) {
        port_error_handler();
    }

    return std::make_pair(true, std::move(packet));
}

void PacketHandler::port_error_handler() {
    serial.close();
    serial.open("/dev/cu.usbserial-00000000", 115200);
    if (serial.errorStatus()) {
        //TODO: throw something here
    }
}


