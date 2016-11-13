#pragma once

#include "nonmoveable.h"
#include "geometry.h"
#include "occupancy_grid.h"
#include "scanline.h"

#include <vector>
#include <opencv2/core.hpp>
#include <boost/range/iterator_range.hpp>

// Build an occupancy grid map based on scan matching. 
// Instead of relying on the odometry data alone, 
// this algorithm estimates the robot position by matching 
// the measured obstacles in each SScanLine against the 
// existing occupancy grid.
struct COccupancyGridWithObstacleList : COccupancyGridBaseT<COccupancyGridWithObstacleList> {
    COccupancyGridWithObstacleList() noexcept;
    COccupancyGridWithObstacleList(COccupancyGridWithObstacleList const& occgrid) noexcept;
    COccupancyGridWithObstacleList(COccupancyGridWithObstacleList&& occgrid) noexcept;
    COccupancyGridWithObstacleList& operator=(COccupancyGridWithObstacleList const& occgrid) noexcept;
    COccupancyGridWithObstacleList& operator=(COccupancyGridWithObstacleList&& occgrid) noexcept;

    rbt::pose<double> fit(rbt::pose<double> const& poseTransform, std::vector<rbt::point<double>> const& vecptfTemplate) const;

    void finishedUpdate();

    cv::Mat ObstacleMap() const;
    cv::Mat ObstacleMapWithPoses(std::vector<rbt::pose<double>> const& vecpose) const;

    template<typename Derived> friend struct COccupancyGridBaseT;
    void updateGrid(rbt::point<int> const& pt, double fOdds);
    void updateGridPoly(boost::iterator_range<rbt::point<int> const*> rngpt, double fOdds) {}

    std::vector<rbt::point<double>> m_vecptfOccupied;
    std::size_t m_iEndSorted;
};
 
struct CScanMatchingBase : rbt::nonmoveable {
    CScanMatchingBase();
    void receivedSensorData(SScanLine const& scanline);
    cv::Mat getMap() const;

    std::vector<rbt::pose<double>> const& Poses() const { return m_vecpose; } 

private:
    COccupancyGridWithObstacleList m_occgrid;
    std::vector<rbt::pose<double>> m_vecpose; // history of best poses
}; 

using CScanMatching = SAccumulateScanline<CScanMatchingBase>;