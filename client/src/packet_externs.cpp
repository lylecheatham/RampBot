#include "packet_externs.h"
#include <array>
#include <vector>
#include "FastCRC.h"

FastCRC16 CRC16;

uint16_t packet_externs::get_serial_packet(node_id destination) {
    static std::array<uint16_t, number_node_ids> counts = {0};

    if (destination >= number_node_ids) { return 0; }
    return counts[destination]++;
}

uint16_t packet_externs::packet_get_crc(const char *data, size_t length) {
    return CRC16.ccitt((uint8_t *)data, length);
}

uint16_t packet_externs::packet_get_crc(std::vector<char> data) {
    return CRC16.ccitt((uint8_t *)data.data(), data.size());
}

node_id packet_externs::this_node = micro;
