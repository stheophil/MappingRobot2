#ifndef robot_controller
#define robot_controller

#include "rover.h" // Data structures and configuration data shared with Arduino controller
#include "occupancy_grid.h"
#include "nonmoveable.h"

struct CRobotController : rbt::nonmoveable {
    CRobotController();        
    void receivedSensorData(SSensorData const& data);
    cv::Mat const& getMap();
private:
    COccupancyGrid m_occgrid;
    std::vector<rbt::pose<double>> m_vecpose;
};

#endif // robot_controller
