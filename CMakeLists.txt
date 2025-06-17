cmake_minimum_required(VERSION 3.31)
project(Physics2D)

set(CMAKE_CXX_STANDARD 20)

add_executable(Physics2D
        src/main.cpp
        src/Math.h
        src/Physics.h
)
