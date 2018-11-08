cmake_minimum_required(VERSION 3.5)

# CMake needs to know this project has assembly, C, and C++
project(RampBotHost C CXX)

# Find Boost
find_package(Boost
        1.62.0
        REQUIRED
        COMPONENTS
        system
)

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

# Make sure cmake gets the right boost version
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_CXX_LINKER_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

macro(get_WIN32_WINNT version)
    if (WIN32 AND CMAKE_SYSTEM_VERSION)
        set(ver ${CMAKE_SYSTEM_VERSION})
        string(REPLACE "." "" ver ${ver})
        string(REGEX REPLACE "([0-9])" "0\\1" ver ${ver})

        set(${version} "0x${ver}")
    endif()
endmacro()

get_WIN32_WINNT(ver)
add_definitions(-D_WIN32_WINNT=${ver})

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
        ${Boost_LIBRARIES}
        )

target_include_directories(RampBotHost PRIVATE
        "src/"
        "../share/"
        )


