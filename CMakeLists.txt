# Test CMake version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set( CMAKE_CXX_STANDARD 14)

# The name of the project
PROJECT(Main)

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

#Same, but for openCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

#Same as above, but for bachelorPro
include_directories($ENV{BACH_ROOT})
include_directories($ENV{URCON_ROOT})
include_directories($ENV{SDHCON_ROOT})
include_directories($ENV{VISION_ROOT})
include_directories($ENV{TIAFC_ROOT})
include_directories($ENV{COORDS_ROOT})
include_directories($ENV{ANALYT_ROOT})


# Set the output dir for generated binaries
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)

# Add classes
add_library(URControl $ENV{URCON_ROOT}/URControl.cpp)
add_library(SDHControl $ENV{SDHCON_ROOT}/SDHControl.cpp)
add_library(Vision $ENV{VISION_ROOT}/Vision.cpp)
add_library(TIAFC $ENV{TIAFC_ROOT}/TIAFC.cpp)
add_library(Coords $ENV{COORDS_ROOT}/Coords.cpp)
add_library(AnalytGrasp $ENV{ANALYT_ROOT}/AnalytGrasp.cpp)

#Add target executeable
add_executable( Main Main.cpp $ENV{SDHCON_ROOT}/sdhoptions.cpp)

#Link all libraries and classes
target_link_libraries(AnalytGrasp ${OpenCV_LIBS})
target_link_libraries(AnalytGrasp Coords)

target_link_libraries(TIAFC ${OpenCV_LIBS})
target_link_libraries(TIAFC Coords)

target_link_libraries(Vision TIAFC)
target_link_libraries(Vision AnalytGrasp)

target_link_libraries(URControl ${ROBWORK_LIBRARIES})
target_link_libraries(URControl ${ROBWORKHARDWARE_LIBRARIES})

target_link_libraries(SDHControl ${ROBWORK_LIBRARIES})
target_link_libraries(SDHControl ${ROBWORKHARDWARE_LIBRARIES})
target_link_libraries(SDHControl ${SDH_LIBRARY})
target_link_libraries(SDHControl ${PEAK_LIBRARY})
target_link_libraries(SDHControl rwhw_sdh)

target_link_libraries(Main URControl)
target_link_libraries(Main SDHControl)
target_link_libraries(Main Vision )




