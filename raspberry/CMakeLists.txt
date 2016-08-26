project (robot_raspi)
cmake_minimum_required (VERSION 2.6)

# Set BOOST_ROOT and OpenCV_DIR environment variables
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CXX_STANDARD 11)

set(Boost_DEBUG ON)
find_package(Boost 1.55.0 REQUIRED COMPONENTS system program_options)
find_package(OpenCV REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("../arduino/src")

add_executable(robot 
	occupancy_grid.cpp
	robot_controller.cpp
	main.cpp
	error_handling.cpp
	robot_configuration.cpp
	particle_slam.cpp)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

target_compile_options(robot PRIVATE --std=c++1y -DBOOST_RANGE_ENABLE_CONCEPT_ASSERT=0 -Wall -Wno-unused-local-typedef)

target_link_libraries( robot 
	${OpenCV_LIBS} 
	${Boost_LIBRARIES}
	pthread
)
