cmake_minimum_required(VERSION 3.0.2)

# Project name
project(socket_utils LANGUAGES C CXX)

# Add library
add_library(socket_utils STATIC
    socket_utils.cpp
)

# Include directories
target_include_directories(socket_utils PUBLIC ./)

# Set C++ standard
set_target_properties(socket_utils PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
)
