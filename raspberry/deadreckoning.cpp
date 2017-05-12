#include "deadreckoning.h"

#include "math.h"
#include "geometry.h"
#include "nonmoveable.h"
#include "occupancy_grid.h" 
#include "occupancy_grid.inl" 
#include "robot_configuration.h"

#include <vector>
#include <chrono>
#include <boost/algorithm/cxx11/all_of.hpp>

CDeadReckoningMapping::CDeadReckoningMapping()
{}
        
void CDeadReckoningMapping::receivedSensorData(SOdometryData const& odom) {
    auto const posePrev = m_vecpose.empty() 
        ? rbt::pose<double>(rbt::point<double>::zero(), 0)
        : m_vecpose.back();
    
    auto const poseNew = UpdatePose(posePrev, odom);

    m_vecpose.emplace_back(poseNew);
}

// void CDeadReckoningMapping::receivedSensorData(SScanLine const& scanline) {
//     m_occgrid.update(m_vecpose.back(), rbt::rad(data.m_nAngle), data.m_nDistance);    
// }

cv::Mat const& CDeadReckoningMapping::getMap() {
    return m_occgrid.ObstacleMap();
}
