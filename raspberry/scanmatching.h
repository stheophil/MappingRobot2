#pragma once

#include "nonmoveable.h"
#include "geometry.h"
#include "occupancy_grid.h"
#include "scanline.h"

#include <vector>
#include <opencv2/core.hpp>

// Build an occupancy grid map based on scan matching. 
// Instead of relying on the odometry data alone, 
// this algorithm estimates the robot position by matching 
// the measured obstacles in each SScanLine against the 
// existing occupancy grid. 
struct CScanMatchingBase : rbt::nonmoveable {
    CScanMatchingBase();
    void receivedSensorData(SScanLine const& scanline);
    cv::Mat getMap() const;

    std::vector<rbt::pose<double>> const& Poses() const { return m_vecpose; } 

private:
    COccupancyGrid m_occgrid;
    std::vector<rbt::pose<double>> m_vecpose; // history of best poses
}; 

using CScanMatching = SAccumulateScanline<CScanMatchingBase>;