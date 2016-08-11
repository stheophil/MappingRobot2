#include "particle_slam.h"
#include "robot_configuration.h"
#include "error_handling.h"

#include <boost/range/algorithm/max_element.hpp>
#include <boost/range/adaptor/transformed.hpp>

#include <opencv2/imgproc.hpp>
#include <iostream>
#include <random>

/////////////////////
// SScanLine

bool SScanLine::add(SSensorData const& data) {
    if(m_vecscan.empty()) {
        m_vecscan.emplace_back(UpdatePose(rbt::pose<double>::zero(), data), data.m_nAngle, data.m_nDistance);
        return true;
    } else {
        auto CompareAngle = [](int lhs, int rhs) {
            if(lhs<rhs) return -1;
            if(rhs<lhs) return 1;
            return 0;
        };
        auto const nCompare = CompareAngle(m_vecscan.front().m_nAngle, m_vecscan.back().m_nAngle);
        auto const nCompareOther = CompareAngle(m_vecscan.back().m_nAngle, data.m_nAngle);
        if(nCompare==0 || nCompareOther==0 || nCompare==nCompareOther) { 
            // Lidar didn't change direction, add sensor to scanline
            m_vecscan.emplace_back(UpdatePose(m_vecscan.back().m_pose, data), data.m_nAngle, data.m_nDistance);
            return true;
        } else {
            return false; // start new scanline
        }
    }
}

rbt::size<double> SScanLine::translation() const {
    return rbt::size<double>(m_vecscan.back().m_pose.m_pt);
}

double SScanLine::rotation() const {
    return m_vecscan.back().m_pose.m_fYaw;
}

void SScanLine::clear() {
    m_vecscan.clear();
}

/////////////////////
// SParticle
SParticle::SParticle() 
    : m_pose(rbt::pose<double>::zero()),
    m_matLikelihood(400, 400, CV_32FC1, cv::Scalar(0)),
    m_occgrid(rbt::size<int>(400, 400), /*nScale*/ 5) 
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
            auto ptn = m_occgrid.toGridCoordinates(pt);
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
CParticleSLAM::CParticleSLAM(int cParticles)
    : m_vecparticle(cParticles), m_vecparticleTemp(cParticles), m_itparticle(m_vecparticle.end())
{}

static std::random_device s_rd;
bool CParticleSLAM::receivedSensorData(SSensorData const& data) {
    // TODO: Ignore data when robot is not moving for a long time
    
    // add to scan line
    if(!m_scanline.add(data)) { 
        // if scanline full, update all particles,
        LOG("Update particles");
        LOG("t = (" << m_scanline.translation().x << 
            ";" << m_scanline.translation().y << ") "
            "r = " << m_scanline.rotation());

        int i = 0; 
        double fWeightTotal = 0.0;
        boost::for_each(m_vecparticle, [&](SParticle& p) {
            p.update(m_scanline);
            fWeightTotal += p.m_fWeight;

            LOG("Particle " << i << " -> " <<  
                " pt = (" << p.m_pose.m_pt.x << "; " << p.m_pose.m_pt.y << ") " <<
                " yaw =  " << p.m_pose.m_fYaw << 
                " w = " << p.m_fWeight);
            ++i;
        });

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

        m_itparticle = boost::max_element(
            boost::adaptors::transform(m_vecparticle, std::mem_fn(&SParticle::m_fWeight))
        ).base();
        m_vecpose.emplace_back(m_itparticle->m_pose);

        m_scanline.clear();
        m_scanline.add(data);
        return true;
    }
    return false;
}

cv::Mat CParticleSLAM::getMap() const {
    ASSERT(m_itparticle!=m_vecparticle.end());
    cv::Mat m = m_itparticle->m_occgrid.ObstacleMap();
    rbt::point<int> ptnPrev = m_itparticle->m_occgrid.toGridCoordinates(rbt::point<double>(0, 0));
    boost::for_each(m_vecpose, [&](rbt::pose<double> const& pose) {
        auto ptnGrid = m_itparticle->m_occgrid.toGridCoordinates(pose.m_pt);
        cv::line(m, ptnPrev, ptnGrid, cv::Scalar(0));
        ptnPrev = ptnGrid;
    });
    return m;
}