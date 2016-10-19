#pragma once

#include "rover.h" // Data structures and configuration data shared with Arduino controller
#include "occupancy_grid.h"
#include "nonmoveable.h"

struct CDeadReckoningMapping : rbt::nonmoveable {
    CDeadReckoningMapping();        
    void receivedSensorData(SSensorData const& data);
    cv::Mat const& getMap();
private:
    COccupancyGrid m_occgrid;
    std::vector<rbt::pose<double>> m_vecpose;
};
