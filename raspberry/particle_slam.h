#pragma once

#include "geometry.h"
#include "occupancy_grid.h"

#include <vector>
#include <opencv2/core.hpp>
#include "scanline.h"

// Simple particle filter algorithm as described 
// in Thrun et al "Probabilistic Robotics" p 478
struct SParticle {
    rbt::pose<double> m_pose;
    
    double m_fWeight;
    cv::Mat m_matLikelihood; // likelihood field
    
    COccupancyGrid m_occgrid;
    
    SParticle();
    SParticle(SParticle const& p);
    SParticle& operator=(SParticle const& p);

    void update(SScanLine const& scanline);
};

struct CParticleSlamBase : rbt::nonmoveable {
    CParticleSlamBase(int cParticles = 100);
    void receivedSensorData(SScanLine const& scanline);
    cv::Mat getMap() const;

    std::vector<rbt::pose<double>> const& Poses() const { return m_vecpose; } 

private:
    std::vector<SParticle> m_vecparticle;
    std::vector<SParticle>::const_iterator m_itparticleBest;
    std::vector<rbt::pose<double>> m_vecpose; // history of best poses

    std::vector<SParticle> m_vecparticleTemp;
}; 