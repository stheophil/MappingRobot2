#ifndef robot_configuration
#define robot_configuration

#include "rover.h"
#include "geometry.h"

// Robot configuration
const int c_nRobotWidth = 30; // cm
const int c_nRobotHeight = 30; // cm

double InitialYaw(SSensorData const& sensordata);
rbt::pose UpdatePose(rbt::pose const& pose, SSensorData const& sensordata); 
void ForEachCell(rbt::point<int> const& ptnGrid, double fYaw, SSensorData const& sensordata, 
    cv::Mat const& matGrid, int nScale, 
    std::function<void(rbt::point<int> const&, float)> UpdateGrid  
);

const float c_fOccupancyRover = -100; // value in occupancy grid of positions occupied by rover itself

#endif // robot_configuration