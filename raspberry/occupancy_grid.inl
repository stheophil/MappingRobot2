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

#include <boost/range/iterator_range.hpp>
#include <boost/range/size.hpp>
#include <opencv2/imgproc.hpp>

template<typename Derived>
COccupancyGridBaseT<Derived>::COccupancyGridBaseT()
:   m_matfMapLogOdds(c_nMapExtent, c_nMapExtent, CV_32FC1, cv::Scalar(0.0f))
{}

template<typename Derived>
COccupancyGridBaseT<Derived>::COccupancyGridBaseT(COccupancyGridBaseT<Derived> const& occgrid) 
:   m_matfMapLogOdds(occgrid.m_matfMapLogOdds.clone())
{}

template<typename Derived>
COccupancyGridBaseT<Derived>::COccupancyGridBaseT(COccupancyGridBaseT<Derived>&& occgrid) 
:   m_matfMapLogOdds(occgrid.m_matfMapLogOdds)
{}

template<typename Derived>
COccupancyGridBaseT<Derived>& COccupancyGridBaseT<Derived>::operator=(COccupancyGridBaseT<Derived> const& occgrid) { 
    m_matfMapLogOdds=occgrid.m_matfMapLogOdds.clone();
    return *this;
}

template<typename Derived>
COccupancyGridBaseT<Derived>& COccupancyGridBaseT<Derived>::operator=(COccupancyGridBaseT<Derived>&& occgrid) { 
    m_matfMapLogOdds=occgrid.m_matfMapLogOdds;
    return *this;
}

template<typename Derived>
bool COccupancyGridBaseT<Derived>::occupied(rbt::point<int> const& pt) const {
	assert(is_inside(pt));
    return c_fFreeThreshold<m_matfMapLogOdds.at<float>(pt.y, pt.x);
}

template<typename Derived>
bool COccupancyGridBaseT<Derived>::is_inside(rbt::point<int> const& pt) const {
	auto const sz = m_matfMapLogOdds.size();
	return 0<=pt.x && 0<=pt.y && pt.x < sz.width && pt.y < sz.height;
}

template<typename Derived>
void COccupancyGridBaseT<Derived>::internalUpdatePerObstacle(rbt::point<double> const& ptf, rbt::point<double> const& ptfObstacle) {
    cv::LineIterator itpt(
        m_matfMapLogOdds, 
        ToGridCoordinate(ptf), 
        ToGridCoordinate(ptfObstacle)
    );
    for(int i = 0; i < itpt.count; i++, ++itpt) {    	
        auto const pt = rbt::point<int>(itpt.pos());
        assert(is_inside(pt)); // LineIterator clips to line boundaries

        auto const fDeltaValue = i<itpt.count-1 
            ? c_fFreeDelta // free
            : c_fOccupiedDelta; // occupied  

        auto const fOdds = m_matfMapLogOdds.at<float>(pt.y, pt.x) + fDeltaValue;
        m_matfMapLogOdds.at<float>(pt.y, pt.x) = fOdds;

        static_cast<Derived*>(this)->updateGrid(pt, fOdds);	
    }
}

template<typename Derived>
void COccupancyGridBaseT<Derived>::internalUpdatePerPose(rbt::pose<double> const& pose) {
    static_cast<Derived*>(this)->updateGridPoly(
    	RenderRobotPose(m_matfMapLogOdds, pose, cv::Scalar(c_fOccupancyRover)), 
    	c_fOccupiedDelta
    );
}

template<typename Derived>
void COccupancyGridBaseT<Derived>::update(rbt::pose<double> const& pose, double fRadAngle, int nDistance) {
    internalUpdatePerObstacle(pose.m_pt, Obstacle(pose, fRadAngle, nDistance));
    internalUpdatePerPose(pose);
}

template<typename Derived>
void COccupancyGridBaseT<Derived>::update(rbt::pose<double> const& pose, std::vector<rbt::point<double>> const& vecptf) {
    boost::for_each(vecptf, [&](auto const& ptf) {
        internalUpdatePerObstacle(pose.m_pt, ptf);
    });
    internalUpdatePerPose(pose);
}
