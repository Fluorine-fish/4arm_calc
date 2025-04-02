cmake_minimum_required(VERSION 3.29)
project(4arm_calc C)

set(CMAKE_C_STANDARD 11)

include_directories(.)

add_executable(4arm_calc
    Calc.c
    Calc.h)
