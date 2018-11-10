/*******************************************************************************
 *
 * FILENAME: config_packet.h
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

#pragma once

#include "packet_data.h"


class Config_Packet : public Packet_Data {
public:
    enum message_type : uint8_t {
        heartbeat = 0,
        wakeup,
        reset,
        pause,
        resume,

        // Leave at end
        num_message_type
    };

    Config_Packet();
    Config_Packet(message_type message);

    static inline Packet_Data *create_packet() { return new Config_Packet(); }

    packet_error deserialize(const char *data, uint8_t size) override;
    std::pair<packet_error, std::vector<char>> serialize() override;
    packet_id get_packet_id() override;


    void set_message(message_type);
    message_type get_message();


private:
    message_type message;
};
