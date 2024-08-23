cmake_minimum_required(VERSION 3.29)
project(sonar_distance_git)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(sonar_distance_git
    arduino_secrets.h
    sonar_distance_git.ino)
