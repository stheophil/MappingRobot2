#pragma once

#include "rover.h"
#include "geometry.h"

// Robot configuration
// All robot parameters are configurable here, as well as global parameters
// such as the map dimensions, the map scaling factor etc. 

// I've also collected the customization points for the implemented algorithms here, e.g.,
// for the particle filter

// Motion model
double InitialYaw(SSensorData const& sensordata);
rbt::pose<double> UpdatePose(rbt::pose<double> const& pose, SSensorData const& sensordata); 

// Occupancy grid
int constexpr c_nScale = 5; // 5cm / px
int constexpr c_nMapExtent = 400; // px ~ 20m
double constexpr c_fOccupiedDelta = 20;
double constexpr c_fFreeDelta = -0.5;
    
rbt::point<int> ToGridCoordinate(rbt::point<double> const& pt);
rbt::pose<int> ToGridCoordinate(rbt::pose<double> const& pose);
    
template<typename T>
rbt::point<T> ToWorldCoordinate(rbt::point<T> const& pt) {
    return (pt - rbt::size<T>(c_nMapExtent, c_nMapExtent)/2) * c_nScale;
}

template<typename T>
rbt::pose<T> ToWorldCoordinate(rbt::pose<T> const& pose) {
    return rbt::pose<T>(ToWorldCoordinate(pose.m_pt), pose.m_fYaw);
}

rbt::point<double> Obstacle(rbt::pose<double> const& pose, double fRadAngle, int nDistance);


const int c_nRobotWidth = 30; // cm
const int c_nRobotHeight = 30; // cm
const float c_fOccupancyRover = -100; // value in occupancy grid of positions occupied by rover itself

// Particle filter
struct SScanLine;
rbt::pose<double> sample_motion_model(rbt::pose<double> const& pose, rbt::size<double> const& szf, double fRadAngle);
double measurement_model_map(rbt::pose<double> const& pose, SScanLine const& scanline, std::function<double (rbt::point<double>)> Distance);
