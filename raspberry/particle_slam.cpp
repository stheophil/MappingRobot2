#include "particle_slam.h"
#include "robot_configuration.h"
#include "error_handling.h"

#include <boost/range/algorithm/max_element.hpp>
#include <boost/range/adaptor/transformed.hpp>

#include <opencv2/imgproc.hpp>
#include <iostream>
#include <random>
#include <future>

/////////////////////
// SParticle
SParticle::SParticle() 
    : m_pose(rbt::pose<double>::zero()),
    m_matLikelihood(c_nMapExtent, c_nMapExtent, CV_32FC1, cv::Scalar(0))
{}

SParticle::SParticle(SParticle const& p)
    : m_pose(p.m_pose),
     m_matLikelihood(p.m_matLikelihood.clone()),
     m_occgrid(p.m_occgrid)
{}

SParticle& SParticle::operator=(SParticle const& p) {
    m_pose = p.m_pose;
    m_matLikelihood = p.m_matLikelihood.clone();
    m_occgrid = p.m_occgrid;
    return *this;
}

void SParticle::update(SScanLine const& scanline) {
    m_pose = sample_motion_model(m_pose, scanline.translation(), scanline.rotation());

    // OPTIMIZE: Match fewer points
    m_fWeight = measurement_model_map(m_pose, scanline, 
        [this](rbt::point<double> const& pt) {
            auto const ptn = ToGridCoordinate(pt);
            return static_cast<double>(m_matLikelihood.at<float>(ptn.y, ptn.x));
        });

    // OPTIMIZE: Recalculate occupancy grid after resampling?
    // OPTIMIZE: m_occgrid.update also sets occupancy of robot itself each time
    scanline.ForEachScan(m_pose, 
        [&](rbt::pose<double> const& poseScan, double fAngle, int nDistance) {
            m_occgrid.update(poseScan, fAngle, nDistance);
        });

    cv::distanceTransform(m_occgrid.ObstacleMap(), m_matLikelihood, CV_DIST_L2, 3); 
}

///////////////////////
// SParticleSLAM
CParticleSlamBase::CParticleSlamBase(int cParticles)
    : m_vecparticle(cParticles), m_itparticleBest(m_vecparticle.end()), m_vecparticleTemp(cParticles) 
{}

static std::random_device s_rd;
void CParticleSlamBase::receivedSensorData(SScanLine const& scanline) {
    // TODO: Ignore data when robot is not moving for a long time
    
    // if scanline full, update all particles,
    LOG("Update particles");
    LOG("t = (" << scanline.translation().x << 
        ";" << scanline.translation().y << ") "
        "r = " << scanline.rotation());

    std::vector<std::future<double>> vecfuture;
    boost::for_each(m_vecparticle, [&](SParticle& p) {
        vecfuture.emplace_back( 
            std::async(std::launch::async | std::launch::deferred,
                [&] {
                    p.update(scanline);
                    return p.m_fWeight;
                }
            ));
    }); 

    double fWeightTotal = 0.0;
    for(int i=0; i<vecfuture.size(); ++i) {
        fWeightTotal += vecfuture[i].get();

        auto const& p = m_vecparticle[i];
        LOG("Particle " << i << " -> " <<  
            " pt = (" << p.m_pose.m_pt.x << "; " << p.m_pose.m_pt.y << ") " <<
            " yaw =  " << p.m_pose.m_fYaw << 
            " w = " << p.m_fWeight);
    }

    // Resampling
    // Thrun, Probabilistic robotics, p. 110
    auto const fStepSize = fWeightTotal/m_vecparticle.size();
    auto const r = std::uniform_real_distribution<double>(0.0, fStepSize)(s_rd);
    auto c = m_vecparticle.front().m_fWeight;

    auto itparticleOut = m_vecparticleTemp.begin();
    for(int i = 0, m = 0; m<m_vecparticle.size(); ++m) {
        auto const u = r + m * fStepSize;
        while(c<u) {
            ++i;
            c += m_vecparticle[i].m_fWeight;
        }
        LOG("Sample particle " << i);
        *itparticleOut = m_vecparticle[i];
        ++itparticleOut;
    }
    std::swap(m_vecparticle, m_vecparticleTemp);

    m_itparticleBest = boost::max_element(
        boost::adaptors::transform(m_vecparticle, std::mem_fn(&SParticle::m_fWeight))
    ).base();
    m_vecpose.emplace_back(m_itparticleBest->m_pose);
}

cv::Mat CParticleSlamBase::getMap() const {
    ASSERT(m_itparticleBest!=m_vecparticle.end());
    cv::Mat m = m_itparticleBest->m_occgrid.ObstacleMap();
    rbt::point<int> ptnPrev = ToGridCoordinate(rbt::point<double>(0, 0));
    boost::for_each(m_vecpose, [&](rbt::pose<double> const& pose) {
        auto const ptnGrid = ToGridCoordinate(pose.m_pt);
        cv::line(m, ptnPrev, ptnGrid, cv::Scalar(0));
        ptnPrev = ptnGrid;
    });
    return m;
}