# Author: Jingting Liu
# CMakeLists for Mavlink library
cmake_minimum_required(VERSION 3.2.0)

PROJECT(Mavlink)

include_directories(${CMAKE_CURRENT_LIST_DIR})
include_directories(${CMAKE_CURRENT_LIST_DIR}/Mavlink2_lib)
#add_subdirectory(Mavlink2_lib)
#set_target_properties(Mavlink2_lib PROPERTIES LINKER_LANGUAGE CXX)

#ADD_LIBRARY(DEENCODERS Airside_Functions.cpp)

add_executable(FUNCTIONS Airside_Functions.cpp)
set_target_properties(FUNCTIONS PROPERTIES LINKER_LANGUAGE CXX)
#target_link_libraries(FUNCTIONS LINK_PUBLIC Mavlink2_lib)
