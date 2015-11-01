cmake_minimum_required (VERSION 2.6)

# Set BOOST_ROOT and OpenCV_DIR environment variables
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CXX_STANDARD 11)

find_package(Boost 1.59.0 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

project (robot_raspi)
add_executable(robot 
	edge_following_strategy.cpp
	occupancy_grid.cpp
	robot_controller.cpp)

target_compile_options( robot PRIVATE --std=c++1y PRIVATE -stdlib=libstdc++)
target_link_libraries( robot ${OpenCV_LIBS} )
