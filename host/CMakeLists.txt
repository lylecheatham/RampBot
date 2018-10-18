cmake_minimum_required(VERSION 3.5)

# CMake needs to know this project has assembly, C, and C++
project(RampBotHost C CXX)

# Find Boost
find_package(Boost
    1.67.0
    REQUIRED
)

# Make sure cmake gets the right boost version
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_CXX_LINKER_FLAGS} -lboost_system")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

add_subdirectory(../share/packet ${CMAKE_CURRENT_BINARY_DIR}/packet)


# Executable and source
add_executable(RampBotHost
        "src/main.cpp"
        "src/BufferedAsyncSerial.cpp"
        "src/AsyncSerial.cpp"
        "src/robot.cpp"
        "src/microcontroller.cpp"
        "src/PacketHandler.cpp"
)

target_link_libraries(RampBotHost PRIVATE
        packet
        )

target_include_directories(RampBotHost PRIVATE
        "src/"
        "../share/"
        )


