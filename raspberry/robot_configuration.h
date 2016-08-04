#ifndef robot_configuration
#define robot_configuration

#include "rover.h"
#include "geometry.h"

// Robot configuration

// Motion model
double InitialYaw(SSensorData const& sensordata);
rbt::pose<double> UpdatePose(rbt::pose<double> const& pose, SSensorData const& sensordata); 

// Occupancy grid
void ForEachCell(rbt::pose<int> const& poseGrid, 
    double fAngle, int nDistance, 
    cv::Mat const& matGrid, int nScale, 
    std::function<void(rbt::point<int> const&, float)> UpdateGrid  
);

const int c_nRobotWidth = 30; // cm
const int c_nRobotHeight = 30; // cm
const float c_fOccupancyRover = -100; // value in occupancy grid of positions occupied by rover itself

#endif // robot_configuration