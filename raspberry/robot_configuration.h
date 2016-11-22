#pragma once

#include "rover.h"
#include "geometry.h"
#include "scanline.h"

// Robot configuration
// All robot parameters are configurable here, as well as global parameters
// such as the map dimensions, the map scaling factor etc. 

// I've also collected the customization points for the implemented algorithms here, e.g.,
// for the particle filter

// Motion model
double InitialYaw(SSensorData const& sensordata);
rbt::pose<double> UpdatePose(rbt::pose<double> const& pose, SSensorData const& sensordata); 

// Occupancy grid
int constexpr c_nScale = 5; // 5cm / px
int constexpr c_nMapExtent = 400; // px ~ 20m
double constexpr c_fOccupiedDelta = 2;
double constexpr c_fFreeDelta = -0.5;
double constexpr c_fFreeThreshold = 1;

rbt::point<int> ToGridCoordinate(rbt::point<double> const& pt);
rbt::pose<int> ToGridCoordinate(rbt::pose<double> const& pose);
    
template<typename T>
rbt::point<T> ToWorldCoordinate(rbt::point<T> const& pt) {
    return (pt - rbt::size<T>(c_nMapExtent, c_nMapExtent)/2) * c_nScale;
}

template<typename T>
rbt::pose<T> ToWorldCoordinate(rbt::pose<T> const& pose) {
    return rbt::pose<T>(ToWorldCoordinate(pose.m_pt), pose.m_fYaw);
}

rbt::point<double> Obstacle(rbt::pose<double> const& pose, double fRadAngle, double nDistance);


const int c_nRobotWidth = 30; // cm
const int c_nRobotHeight = 30; // cm
const float c_fOccupancyRover = -100; // value in occupancy grid of positions occupied by rover itself

// Particle filter
rbt::pose<double> sample_motion_model(rbt::pose<double> const& pose, rbt::size<double> const& szf, double fRadAngle);
double measurement_model_map(rbt::pose<double> const& pose, SScanLine const& scanline, std::function<double (rbt::point<double>)> Distance);

const double c_fSqrt2 = std::sqrt(2);

template<typename TOccupancyGrid>
double log_likelihood_field(rbt::pose<double> const& pose, SScanLine const& scanline, TOccupancyGrid const& occgrid) {
    // See ScanMatcher::likelihoodAndScore in https://svn.openslam.org/data/svn/gmapping/trunk/scanmatcher/scanmatcher.h
    // For every detected obstacle point, iterate over a small kernel to find the obstacle in the map matLogLikelihood
    // closest to the expected position
    double const c_fSensorSigma = 10; // ~ +-10cm
    
    double fLogLikelihood = 0.0;
    
#ifdef ENABLE_LOG
    int nCountObstacle = 0;
#endif
    scanline.ForEachScan(pose, [&](rbt::pose<double> const& poseScan, double fAngle, int nDistance) {
        auto ptfOccupied = Obstacle(poseScan, fAngle, nDistance);
        auto ptfFree = Obstacle(poseScan, fAngle, nDistance - c_nScale*c_fSqrt2);
        
        auto const ptnOccupied = ToGridCoordinate(ptfOccupied);
        auto const ptnFree = ToGridCoordinate(ptfFree);
        
        double fSqrDistBest = std::numeric_limits<double>::max();
        for(int nOffsetX = -1; nOffsetX<2; ++nOffsetX) {
            for(int nOffsetY = -1; nOffsetY<2; ++nOffsetY) {
                rbt::size<int> szn(nOffsetX, nOffsetY);
                
                auto const ptnOccOff = ptnOccupied + szn;
                auto const ptnFreeOff = ptnFree + szn;
                
                // Checking not only for the occupied point but also for the neighboring
                // free point is also taken from gmapping implementation
                if(occgrid.occupied(ptnOccOff) && !occgrid.occupied(ptnFreeOff)) {
                    double fSqrDist = (ptfOccupied - rbt::point<double>(ToWorldCoordinate(ptnOccOff))).SqrAbs();
                    if(fSqrDist < fSqrDistBest) {
                        fSqrDistBest = fSqrDist;
                    }
                }
            }
        }
        
        if(fSqrDistBest<std::numeric_limits<double>::max()) {
            fLogLikelihood+=(-1./c_fSensorSigma)*fSqrDistBest;
            
#ifdef ENABLE_LOG
            ++nCountObstacle;
#endif
        } else {
            fLogLikelihood+=-60./c_fSensorSigma; // FIXME
        }
    });
    
    
#ifdef ENABLE_LOG
    LOG("Matched " << nCountObstacle << " obstacles");
#endif
    return fLogLikelihood;
}
