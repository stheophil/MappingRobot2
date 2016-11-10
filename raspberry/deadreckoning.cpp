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
        
void CDeadReckoningMapping::receivedSensorData(SSensorData const& data) {
    auto const posePrev = m_vecpose.empty() 
        ? rbt::pose<double>(rbt::point<double>::zero(), InitialYaw(data))
        : m_vecpose.back();
    
    auto const poseNew = UpdatePose(posePrev, data);

    m_vecpose.emplace_back(poseNew);
    m_occgrid.update(poseNew, rbt::rad(data.m_nAngle), data.m_nDistance);    
}

cv::Mat const& CDeadReckoningMapping::getMap() {
    return m_occgrid.ObstacleMap();
}
