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
    COccupancyGrid();        
    COccupancyGrid(COccupancyGrid const& occgrid);
    COccupancyGrid& operator=(COccupancyGrid const& occgrid);

    void update(rbt::pose<double> const& pose, double fAngle, int nDistance);
    void update(rbt::pose<double> const& pose, std::vector<rbt::point<double>> const& vecptf);

    cv::Mat const& LogOddsMap() const { return m_matfMapLogOdds; }
    cv::Mat const& ObstacleMap() const { return m_matnMapObstacle; }
    
    cv::Mat ObstacleMapWithPoses(std::vector<rbt::pose<double>> const& vecpose) const;
private:
    void internalUpdatePerObstacle(rbt::point<double> const& ptf, rbt::point<double> const& ptfObstacle);
    void internalUpdatePerPose(rbt::pose<double> const& pose);

    cv::Mat m_matfMapLogOdds;
    cv::Mat m_matnMapObstacle; // thresholded version of m_matfMapLogOdds
};

#endif /* occupancy_grid_h */
