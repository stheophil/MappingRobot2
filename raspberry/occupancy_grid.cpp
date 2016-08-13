//
//  occupancy_grid.cpp
//  robotcontrol2
//
//  Created by Sebastian Theophil on 24.09.15.
//  Copyright © 2015 Sebastian Theophil. All rights reserved.
//

#include "occupancy_grid.h"
#include "robot_configuration.h" // Data structures and configuration data shared with Arduino controller
#include "error_handling.h"

#include <cstdlib>
#include <cmath>
#include <algorithm>

#include <boost/range/size.hpp>
#include <opencv2/imgproc.hpp>

COccupancyGrid::COccupancyGrid()
:   m_matfMapLogOdds(c_nMapExtent, c_nMapExtent, CV_32FC1, cv::Scalar(0.0f)),
    m_matnMapObstacle(c_nMapExtent, c_nMapExtent, CV_8UC1, cv::Scalar(255))
{}

COccupancyGrid::COccupancyGrid(COccupancyGrid const& occgrid) 
:   m_matfMapLogOdds(occgrid.m_matfMapLogOdds.clone()),
    m_matnMapObstacle(occgrid.m_matnMapObstacle.clone())
{}

COccupancyGrid& COccupancyGrid::operator=(COccupancyGrid const& occgrid) { 
    m_matfMapLogOdds=occgrid.m_matfMapLogOdds.clone();
    m_matnMapObstacle=occgrid.m_matnMapObstacle.clone();
    return *this;
}

void COccupancyGrid::update(rbt::pose<double> const& pose, double fRadAngle, int nDistance) {
    ForEachCell(
        pose, 
        fRadAngle, nDistance, 
        m_matfMapLogOdds, 
        std::bind(&COccupancyGrid::toGridCoordinates, this, std::placeholders::_1), 
        [this](rbt::point<int> const& pt, float fDeltaValue) {
            auto const fOdds = m_matfMapLogOdds.at<float>(pt.y, pt.x) + fDeltaValue;
            m_matfMapLogOdds.at<float>(pt.y, pt.x) = fOdds;
            // Calculating the greyscale map is pretty expensive
            // If we ever need a non-binary version, a lookup table
            // would be useful instead of this:
            // auto const nColor = rbt::numeric_cast<std::uint8_t>(1.0 / ( 1.0 + std::exp( fOdds )) * 255);
            m_matnMapObstacle.at<std::uint8_t>(pt.y, pt.x) = 0 < fOdds ? 0 : 255;
        }
    );
    
    // Clear position of robot itself
    rbt::size<double> const szfHalfSize(c_nRobotWidth/2.0, c_nRobotHeight/2.0);
    cv::Point const apt[] = {
        toGridCoordinates(pose.m_pt - szfHalfSize.rotated(pose.m_fYaw)),
        toGridCoordinates(pose.m_pt + rbt::size<double>(szfHalfSize.x, -szfHalfSize.y).rotated(pose.m_fYaw)),
        toGridCoordinates(pose.m_pt + szfHalfSize.rotated(pose.m_fYaw)),
        toGridCoordinates(pose.m_pt + rbt::size<double>(-szfHalfSize.x, szfHalfSize.y).rotated(pose.m_fYaw))
    };
    cv::fillConvexPoly(m_matfMapLogOdds, apt, boost::size(apt), cv::Scalar(c_fOccupancyRover));
    cv::fillConvexPoly(m_matnMapObstacle, apt, boost::size(apt), cv::Scalar(255));
}

rbt::point<int> COccupancyGrid::toGridCoordinates(rbt::point<double> const& pt) const {
    return rbt::point<int>(pt/c_nScale) + rbt::size<int>(c_nMapExtent, c_nMapExtent)/2;
}

rbt::point<int> COccupancyGrid::toWorldCoordinates(rbt::point<int> const& pt) const {
    return (pt - rbt::size<int>(c_nMapExtent, c_nMapExtent)/2) * c_nScale;
}
