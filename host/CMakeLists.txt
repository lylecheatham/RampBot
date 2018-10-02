cmake_minimum_required(VERSION 3.5)

# CMake needs to know this project has assembly, C, and C++
project(RampBotHost C CXX)

add_executable(RampBotHost
    "src/main.cpp"
)
