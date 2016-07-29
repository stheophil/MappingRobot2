#ifndef robot_controller
#define robot_controller

#include "rover.h" // Data structures and configuration data shared with Arduino controller
#include "occupancy_grid.h"
#include "nonmoveable.h"

enum class map {
    occupancy,
    eroded
};

struct CRobotController : rbt::nonmoveable {
    CRobotController();        
    void receivedSensorData(SSensorData const& data);
    cv::Mat const& getMap(map m);
private:
    COccupancyGrid m_occgrid;
    std::vector<rbt::pose> m_vecpose;
};

#endif // robot_controller
