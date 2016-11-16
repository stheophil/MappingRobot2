#pragma once

#include "geometry.h"
#include "occupancy_grid.h"

#include <vector>
#include <opencv2/core.hpp>
#include "scanline.h"
#include "scanmatching.h"

// Simple particle filter algorithm as described 
// in Thrun et al "Probabilistic Robotics" p 478
struct SFastSlamParticle {
    rbt::pose<double> m_pose;    
    double m_fLogWeight;
    double m_fWeight;
    
    COccupancyGridWithObstacleList m_occgrid;
    
    SFastSlamParticle();
    void updatePose(SScanLine const& scanline);
    void updateMap(SScanLine const& scanline);
};

struct CFastParticleSlamBase : rbt::nonmoveable {
    CFastParticleSlamBase(int cParticles = 10);
    void receivedSensorData(SScanLine const& scanline);
    cv::Mat getMap() const;

    std::vector<rbt::pose<double>> const& Poses() const { return m_vecpose; } 

private:
    std::vector<SFastSlamParticle> m_vecparticle;
    std::vector<SFastSlamParticle>::const_iterator m_itparticleBest;
    
    double m_fNEff;
    
    std::vector<rbt::pose<double>> m_vecpose; // history of best poses
}; 

using CFastParticleSlam = SAccumulateScanline<CFastParticleSlamBase>;
