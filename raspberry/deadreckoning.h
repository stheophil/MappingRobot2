#pragma once

#include "rover.h" 
#include "occupancy_grid.h"
#include "nonmoveable.h"
#include "scanline.h"

/*  Builds an occupancy grid using dead reckoning only, i.e.,
    only based on the odometry values. 
    Useful as a baseline comparison.
*/
struct CDeadReckoningMapping : rbt::nonmoveable {
    CDeadReckoningMapping();        
    void receivedSensorData(SOdometryData const& odom);
    void receivedSensorData(SScanLine const& scanline);

    cv::Mat const& getMap();
private:
    COccupancyGrid m_occgrid;
    std::vector<rbt::pose<double>> m_vecpose;
};
