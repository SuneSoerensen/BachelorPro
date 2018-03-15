
# Test CMake version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# The name of the project. 
PROJECT(TutorialCollection)

SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# optionally: point the find package in the direction of the robwork.
SET(RobWork_DIR ~/RobWork/RobWork/cmake)
FIND_PACKAGE(RobWork REQUIRED)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

#Same as above, but for robworkhardware
SET(RobWorkHardware_DIR ~/RobWork/RobWorkHardware/cmake)
FIND_PACKAGE(RobWorkHardware REQUIRED)

INCLUDE_DIRECTORIES( ${ROBWORKHARDWARE_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORKHARDWARE_LIBRARY_DIRS} )

# Set the output dir for generated libraries and binaries
IF(MSVC)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
ELSE()
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
ENDIF()

# And now we add any targets that we want
add_executable(goToInit_example goToInit_example.cpp)
target_link_libraries(goToInit_example ${ROBWORK_LIBRARIES})
target_link_libraries(goToInit_example ${ROBWORKHARDWARE_LIBRARIES})
