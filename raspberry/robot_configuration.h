#ifndef robot_configuration
#define robot_configuration

#include "rover.h"
#include "geometry.h"

// Robot configuration

// Motion model
double InitialYaw(SSensorData const& sensordata);
rbt::pose<double> UpdatePose(rbt::pose<double> const& pose, SSensorData const& sensordata); 

// Occupancy grid
void ForEachCell(
    rbt::pose<double> const& pose, 
    double fRadAngle, int nDistance, 
    cv::Mat const& matGrid, 
    std::function<rbt::point<int> (rbt::point<double>)> ToGridPoint, 
    std::function<void(rbt::point<int> const&, float)> UpdateGrid  
);

const int c_nRobotWidth = 30; // cm
const int c_nRobotHeight = 30; // cm
const float c_fOccupancyRover = -100; // value in occupancy grid of positions occupied by rover itself

// Particle filter
struct SScanLine;
rbt::pose<double> sample_motion_model(rbt::pose<double> const& pose, rbt::size<double> const& szf, double fRadAngle);
double measurement_model_map(rbt::pose<double> const& pose, SScanLine const& scanline, std::function<double (rbt::point<double>)> Distance);

#endif // robot_configuration