cmake_minimum_required(VERSION 3.31)
project(Physics2D)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SFML 2.6 COMPONENTS graphics window system REQUIRED)


add_executable(Physics2D
        src/main.cpp
        src/polyphysics.h
        src/Math.h
)

target_link_libraries(Physics2D
        PRIVATE
        sfml-graphics
        sfml-window
        sfml-system
)