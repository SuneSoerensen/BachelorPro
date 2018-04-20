
# Test CMake version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# The name of the project.
PROJECT(TwoFingerTest)

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

#Same as above, but for SDH
SET(SDH_DIR ~/RobWork/RobWorkHardware/cmake)
FIND_PACKAGE(SDH REQUIRED)

#Same as above, but for PEAKCAN
SET(PEAKCAN_DIR ~/RobWork/RobWorkHardware/cmake)
FIND_PACKAGE(PEAKCAN REQUIRED)

#Same as above, but for bachelorPro
INCLUDE_DIRECTORIES($ENV{BACH_ROOT})
INCLUDE_DIRECTORIES($ENV{URCON_ROOT})
INCLUDE_DIRECTORIES($ENV{SDHCON_ROOT})

# Set the output dir for generated libraries and binaries
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)

# Add your classes REMEMBER TO LINK DEPENDENCIES!
add_library(SDHControl $ENV{SDHCON_ROOT}/SDHControl.cpp)

# And now we add any targets that we want
add_executable(TwoFingerTest TwoFingerTest.cpp $ENV{SDHCON_ROOT}/sdhoptions.cpp)

#And link the libraries
target_link_libraries(SDHControl ${ROBWORK_LIBRARIES})
target_link_libraries(SDHControl ${ROBWORKHARDWARE_LIBRARIES})
target_link_libraries(SDHControl ${SDH_LIBRARY})
target_link_libraries(SDHControl ${PEAK_LIBRARY})
target_link_libraries(SDHControl rwhw_sdh)
target_link_libraries(TwoFingerTest SDHControl)
