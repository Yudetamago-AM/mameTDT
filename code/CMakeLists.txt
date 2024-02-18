#Set the mimum required version of CMake
cmake_minimum_required(VERSION 3.12)

#Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

#Set the project name, the programming language
project(tdc_tdt_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

#Initialize the SDK
pico_sdk_init()

add_subdirectory("main3")
