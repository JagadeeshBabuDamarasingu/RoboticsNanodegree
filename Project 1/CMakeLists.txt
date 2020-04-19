cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(welcome_to_jd_house SHARED script/welcome_to_jd_house.cpp)
target_link_libraries(welcome_to_jd_house ${GAZEBO_LIBRARIES})
