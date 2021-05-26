cmake_minimum_required(VERSION 3.19)

project(Rocket)

add_executable(Rocket rocket.cpp)
add_executable(MovingRocket movingrocket.cpp)

set(CMAKE_CXX_FLAGS "-O2")
