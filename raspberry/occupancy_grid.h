//
//  occupancy_grid.h
//  robotcontrol2
//
//  Created by Sebastian Theophil on 24.09.15.
//  Copyright © 2015 Sebastian Theophil. All rights reserved.
//

#ifndef occupancy_grid_h
#define occupancy_grid_h

#include "rover.h"
#include "nonmoveable.h"
#include "geometry.h"

#include <opencv2/core.hpp>

struct COccupancyGrid {
    COccupancyGrid(rbt::size<int> const& szn, int nScale);        
    COccupancyGrid(COccupancyGrid const&);

    void update(rbt::pose<double> const& pose, double fAngle, int nDistance);
    
    rbt::point<int> toGridCoordinates(rbt::point<double> const& pt) const;
    rbt::point<int> toWorldCoordinates(rbt::point<int> const& pt) const;
    
    cv::Mat const& LogOddsMap() const { return m_matfMapLogOdds; }
    cv::Mat const& ObstacleMap() const { return m_matnMapObstacle; }
    // cv::Mat const& ErodedMap() const { return m_matnMapEroded; }
    
    rbt::size<int> const m_szn;
    int const m_nScale; // cm per pixel
    
private:
    cv::Mat m_matfMapLogOdds;
    cv::Mat m_matnMapObstacle; // thresholded version of m_matfMapLogOdds
    // cv::Mat m_matnMapEroded;
};

#endif /* occupancy_grid_h */
