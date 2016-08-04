#include "robot_controller.h"

#include "math.h"
#include "geometry.h"
#include "nonmoveable.h"
#include "occupancy_grid.h"
#include "edge_following_strategy.h"
#include "robot_configuration.h"

#include <vector>
#include <chrono>
#include <boost/algorithm/cxx11/all_of.hpp>

CRobotController::CRobotController()
    : m_occgrid(rbt::size<int>(400, 400), /*nScale*/5) // = Map of 20m x 20m map
{}
        
void CRobotController::receivedSensorData(SSensorData const& data) {
    // TODO: Queue sensor data, process in parallel

    auto const posePrev = m_vecpose.empty() 
        ? rbt::pose<double>(rbt::point<double>::zero(), InitialYaw(data))
        : m_vecpose.back();
    
    auto const poseNew = UpdatePose(posePrev, data);

    m_vecpose.emplace_back(poseNew);
    m_occgrid.update(poseNew, rbt::rad(data.m_nAngle), data.m_nDistance);    
}

cv::Mat const& CRobotController::getMap() {
    return m_occgrid.GreyscaleMap();
}
