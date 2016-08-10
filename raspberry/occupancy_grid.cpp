//
//  occupancy_grid.cpp
//  robotcontrol2
//
//  Created by Sebastian Theophil on 24.09.15.
//  Copyright © 2015 Sebastian Theophil. All rights reserved.
//

#include "occupancy_grid.h"
#include "robot_configuration.h" // Data structures and configuration data shared with Arduino controller

#include <assert.h>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <unordered_map>

#include <boost/range/size.hpp>
#include <opencv2/imgproc.hpp>

namespace {
    struct SRotatedRect {
        rbt::point<int> m_ptnCenter;
        rbt::size<double> m_szf;
        double m_fAngle;
        
        template<typename Func>
        void for_each_pixel(Func foreach) {
            // Rotate the scaled rectangle.
            // TODO: For numerical precision, it may be better to rotate the rect
            // in world coordinates and then scale.
            // TODO: Use cv::LineIterator instead
            rbt::point<int> apt[] = {
                m_ptnCenter - rbt::size<int>((m_szf/2).rotated(m_fAngle)),
                m_ptnCenter + rbt::size<int>((rbt::size<double>(m_szf.x, -m_szf.y)/2).rotated(m_fAngle)),
                m_ptnCenter + rbt::size<int>((m_szf/2).rotated(m_fAngle)),
                m_ptnCenter + rbt::size<int>((rbt::size<double>(-m_szf.x, m_szf.y)/2).rotated(m_fAngle))
            };
            
            auto rasterize = [](rbt::point<int> ptA, rbt::point<int> ptB, auto foreach) {
                if(ptA.x == ptB.x) {
                    // straight vertical line
                    auto nMin = std::min(ptA.y, ptB.y);
                    auto nMax = std::max(ptA.y, ptB.y);
                    
                    for(int y = nMin; y <= nMax; ++y) {
                        foreach(ptA.x, y);
                    }
                } else {
                    if(ptB.x<ptA.x) {
                        std::swap(ptB, ptA);
                    }
                    
                    auto m = rbt::numeric_cast<double>(ptB.y - ptA.y) / (ptB.x - ptA.x);
                    if(std::abs(m)<=1) { // x-step
                        for(int x = ptA.x; x<= ptB.x; ++x) {
                            foreach(x, rbt::numeric_cast<int>(ptA.y + m * (x - ptA.x)));
                        }
                    } else { // y-step
                        auto nMin = std::min(ptA.y, ptB.y);
                        auto nMax = std::max(ptA.y, ptB.y);
                        for(int y = nMin; y <= nMax; ++y) {
                            foreach(rbt::numeric_cast<int>(ptA.x + (y - ptA.y) / m), y);
                        }
                    }
                }
            };
            
            // TODO: The map could be avoided by sorting the line segments
            std::unordered_map<int, rbt::interval<int>> mapnintvlX;
            for(int i=0; i<boost::size(apt); ++i) {
                rasterize(apt[i], apt[(i+1)%boost::size(apt)], [&](int x, int y) {
                    auto pairitb = mapnintvlX.emplace(y, rbt::interval<int>(x, x));
                    if(!pairitb.second) {
                        pairitb.first->second |= x;
                    }
                });
            }
            boost::for_each(mapnintvlX, [&](auto const& pairnintvlX) {
                for(int x = pairnintvlX.second.begin; x <= pairnintvlX.second.end; ++x) {
                    foreach(rbt::point<int>(x, pairnintvlX.first));
                }
            });
        }
    };
}

COccupancyGrid::COccupancyGrid(rbt::size<int> const& szn, int nScale)
:   m_szn(szn), 
    m_nScale(nScale),
    m_matfMapLogOdds(szn.x, szn.y, CV_32FC1, cv::Scalar(0.0f)),
    m_matnMapGreyscale(szn.x, szn.y, CV_8UC1, cv::Scalar(128))
{
    assert(0==szn.x%2 && 0==szn.y%2);
}

COccupancyGrid::COccupancyGrid(COccupancyGrid const& occgrid) 
:   m_szn(occgrid.m_szn), 
    m_nScale(occgrid.m_nScale), 
    m_matfMapLogOdds(occgrid.m_matfMapLogOdds.clone()),
    m_matnMapGreyscale(occgrid.m_matnMapGreyscale.clone())
{}

void COccupancyGrid::update(rbt::pose<double> const& pose, double fRadAngle, int nDistance) {
    auto UpdateMap = [this](rbt::point<int> const& pt, float fDeltaValue) {
        auto const fOdds = m_matfMapLogOdds.at<float>(pt.y, pt.x) + fDeltaValue;
        m_matfMapLogOdds.at<float>(pt.y, pt.x) = fOdds;
        auto const nColor = rbt::numeric_cast<std::uint8_t>(1.0 / ( 1.0 + std::exp( fOdds )) * 255);
        m_matnMapGreyscale.at<std::uint8_t>(pt.y, pt.x) = nColor;
    };

    ForEachCell(
        pose, 
        fRadAngle, nDistance, 
        m_matfMapLogOdds, 
        std::bind(&COccupancyGrid::toGridCoordinates, this, std::placeholders::_1), 
        UpdateMap
    );
    
    // Clear position of robot itself
    SRotatedRect rectRobot{toGridCoordinates(pose.m_pt), rbt::size<double>(c_nRobotWidth, c_nRobotHeight)/m_nScale, pose.m_fYaw};
    rectRobot.for_each_pixel([&](rbt::point<int> const& pt) {
        UpdateMap(pt, -m_matfMapLogOdds.at<float>(pt.y, pt.x) + c_fOccupancyRover); 
    });
    
    // Erode image
    // A pixel p in imageEroded is marked free when the robot centered at p does not occupy an occupied pixel in self.image
    // i.e. the pixel p has the maximum value of the surrounding pixels inside the diameter defined by the robot's size
    // We overestimate robot size by taking robot diagonal        
    
    // static const int s_nKernelDiameter =
    //     rbt::numeric_cast<int>(std::ceil(std::sqrt(rbt::size<int>(c_nRobotWidth, c_nRobotHeight).SqrAbs()) / m_nScale));
    // static const cv::Mat s_matnKernel = cv::Mat(s_nKernelDiameter, s_nKernelDiameter, CV_8UC1, 1);
    // cv::erode(m_matnMapGreyscale, m_matnMapEroded, s_matnKernel);
}

rbt::point<int> COccupancyGrid::toGridCoordinates(rbt::point<double> const& pt) const {
    return rbt::point<int>(pt/m_nScale) + m_szn/2;
}

rbt::point<int> COccupancyGrid::toWorldCoordinates(rbt::point<int> const& pt) const {
    return (pt - m_szn/2) * m_nScale;
}
