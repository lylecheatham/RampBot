/*******************************************************************************
 *
 * FILENAME: packet_utils.h
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
#include <array>


enum node_id : uint8_t {
    null_node = 0x00,
    external = 0x01,
    micro = 0x02,
    rasp_pi = 0x03,

    // Leave this at end
        number_node_ids
};

enum packet_id : uint8_t {
    null_packet = 0x00,
    packet_response = 0x01,
    config_packet = 0x02,
    time_packet = 0x03,
    sensor_packet = 0x04,
    motor_packet = 0x05,

    // Leave this at end
        number_packet_ids
};

