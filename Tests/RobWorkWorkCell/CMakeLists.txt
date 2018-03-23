
# Test CMake version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# The name of the project. 
PROJECT(SDH_Test)

SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# optionally: point the find package in the direction of the robwork.
SET(RobWork_DIR ~/RobWork/RobWork/cmake)
FIND_PACKAGE(RobWork REQUIRED)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

add_executable(RobWorkTest RobWorkTest.cpp)
target_link_libraries(RobWorkTest ${ROBWORK_LIBRARIES})
