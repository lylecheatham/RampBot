#include <boost/asio/io_service.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/crc.hpp>
#include <array>
#include <string>
#include <iostream>


#include "packet.h"
#include "packet_externs.h"
#include "BufferedAsyncSerial.h"
#include "config_packet.h"

int main(){
    auto test_send = std::unique_ptr<Config_Packet>(new Config_Packet());
    test_send->set_message(Config_Packet::message_type::reset);
    Packet test_init(node_id::external, packet_id::config_packet, std::move(test_send));

    auto test = test_init.serialize();

    if(test.first == p_err_none){
        std::cout << test.second[0] << std::endl;
        std::cout << test.second.size() << std::endl;
    }
    Packet test_receive;
    test_receive.deserialize(&test.second[0]);

    BufferedAsyncSerial port("/dev/cu.usbserial-00000000", 115200);

    if(port.isOpen()){
        port.write(test.second);
    }


    while(1){
        std::string asdf = port.readStringUntil("*");
        std::cout << asdf << std::endl;
    }

}

uint16_t packet_externs::get_serial_packet(node_id destination) {
    static std::array<uint16_t, number_node_ids> counts = {0};

    if (destination >= number_node_ids) {
        return 0;
    }
    return counts[destination]++;
}

uint16_t packet_externs::packet_get_crc(const char *data, size_t length) {
    boost::crc_optimal<16, 0x1021, 0xFFFF, 0x0000, false, false> CRC16_CCITT;
    CRC16_CCITT.process_bytes(data, length);
    return (uint16_t) CRC16_CCITT();
}

uint16_t packet_externs::packet_get_crc(std::vector<char> data) {
    boost::crc_optimal<16, 0x1021, 0xFFFF, 0x0000, false, false> CRC16_CCITT;
    CRC16_CCITT.process_bytes(&data[0], data.size());
    return (uint16_t) CRC16_CCITT();
}

node_id packet_externs::this_node = rasp_pi;