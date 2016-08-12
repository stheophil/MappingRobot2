#pragma once

#include "geometry.h"
#include "occupancy_grid.h"

#include <vector>
#include <opencv2/core.hpp>

struct SScanLine {
    struct SScan {
        SScan(rbt::pose<double> const& pose, int nAngle, int nDistance)
            : m_pose(pose), m_nAngle(nAngle), m_fRadAngle(rbt::rad(nAngle)), m_nDistance(nDistance)
        {}

        rbt::pose<double> m_pose; // accumulated pose change during scan line measurement
        int m_nAngle; // in degrees
        double m_fRadAngle; // in radians
        int m_nDistance;
    };
    std::vector< SScan > m_vecscan;

    // returns false iff lidar angle changed direction, i.e., data was not added
    // and data belongs into new SScanLine
    bool add(SSensorData const& data);
    void clear();

    rbt::size<double> translation() const;
    double rotation() const;

    template<typename Func>
    void ForEachScan(rbt::pose<double> const& pose, Func fn) const {
        auto const& poseLast = m_vecscan.back().m_pose;
        boost::for_each(m_vecscan, [&](SScan const& scan) {
            auto const szf = scan.m_pose.m_pt - poseLast.m_pt;
            auto const fDeltaYaw = scan.m_pose.m_fYaw - poseLast.m_fYaw;
            fn(rbt::pose<double>(pose.m_pt + szf, pose.m_fYaw + fDeltaYaw), scan.m_fRadAngle, scan.m_nDistance);
        });
    }
};

struct SParticle {
    rbt::pose<double> m_pose;
    
    double m_fWeight;
    cv::Mat m_matLikelihood; // likelihood field
    
    COccupancyGrid m_occgrid; // without eroded map 
    
    SParticle();
    SParticle(SParticle const& p);
    SParticle& operator=(SParticle const& p);

    void update(SScanLine const& scanline);
};

struct CParticleSLAM : rbt::nonmoveable {
    CParticleSLAM(int cParticles = 100);
    bool receivedSensorData(SSensorData const& data);
    cv::Mat getMap() const;

    std::vector<rbt::pose<double>> const& Poses() const { return m_vecpose; } 

private:
    std::vector<SParticle> m_vecparticle;
    std::vector<SParticle>::const_iterator m_itparticleBest;
    std::vector<rbt::pose<double>> m_vecpose; // history of best poses
    
    SScanLine m_scanline; // accumulates individual lidar scans

    std::vector<SParticle> m_vecparticleTemp;
}; 