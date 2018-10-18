
# Library and source
add_library(packet
        "packet_handling/packet.cpp"
        "packet_handling/packet_header.cpp"
        "packet_definitions/time_packet.cpp"
        "packet_definitions/sensor_packet.cpp"
        "packet_definitions/config_packet.cpp"
        )

target_include_directories(packet PUBLIC
        "packet_definitions/"
        "packet_handling/"
        )
