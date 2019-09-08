cmake_minimum_required(VERSION 3.13)
project(Test_Software)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(Test_Software
        USBtoCAN.cpp
        USBtoCAN.h cmake-build-debug/main.cpp)
