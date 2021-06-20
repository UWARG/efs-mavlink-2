# Author: Jingting Liu, Dhruv Rawat
# CMakeLists for Mavlink library
cmake_minimum_required(VERSION 3.2.0)

PROJECT(Mavlink2 CXX)

file(GLOB_RECURSE MAVLINK_AIRSIDE_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/Airside_Functions.cpp")
file(GLOB_RECURSE MAVLINK_AIRSIDE_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/Airside_Functions.hpp")

add_library(${PROJECT_NAME} STATIC ${MAVLINK_AIRSIDE_HEADER} ${MAVLINK_AIRSIDE_SOURCE})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Mavlink2_lib
)

# uncomment the following if you want to run the tests with cmake

#add_executable(FUNCTIONS Airside_Functions.cpp)
#set_target_properties(FUNCTIONS PROPERTIES LINKER_LANGUAGE CXX)
#target_link_libraries(FUNCTIONS LINK_PUBLIC Mavlink2_lib)
