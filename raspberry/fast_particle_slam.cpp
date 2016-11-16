#include "fast_particle_slam.h"
#include "robot_configuration.h"
#include "error_handling.h"
#include "occupancy_grid.inl"

#include <boost/range/algorithm/max_element.hpp>
#include <boost/range/adaptor/transformed.hpp>

#include <opencv2/imgproc.hpp>
#include <iostream>
#include <random>
#include <future>

// Based on Grisetti, Stachniss, Burgard 
// "Improving Grid-based SLAM with Rao-Blackwellized Particle Filters by Adaptive Proposals and Selective Resampling"
// and their implementation at https://openslam.org/gmapping.html
SFastSlamParticle::SFastSlamParticle() : m_pose(rbt::pose<double>::zero()) {}

void SFastSlamParticle::updatePose(SScanLine const& scanline) {
    // 1. Update particles with probabilistic motion model
    auto poseSampled = sample_motion_model(m_pose, scanline.translation(), scanline.rotation());

    // 2. If not first update (and optionally: enough distance traveled since last update)
    //    scan match and update particle pose
    m_pose = m_occgrid.fit(poseSampled, scanline);
    
    // 3. Compute likelihood of resulting match
    // gmapping computes log likelihood, also skips distanceTransform and searches
    // in small kernel around expected obstacle
    m_fLogWeight = log_likelihood_field(m_pose, scanline, m_occgrid);
    
    LOG("Update Particle: poseSampled = " << poseSampled << " m_pose = " << m_pose << " m_fLogWeight = " << m_fLogWeight << "\n");
}

void SFastSlamParticle::updateMap(SScanLine const& scanline) {
    scanline.ForEachScan(m_pose, [&](auto const& poseScan, double fAngle, int nDistance) {
        m_occgrid.update(poseScan, fAngle, nDistance);
    });
}

CFastParticleSlamBase::CFastParticleSlamBase(int cParticles) 
    : m_vecparticle(cParticles), m_itparticleBest(m_vecparticle.begin()), m_fNEff(1.0)
{}

static std::random_device s_rd;
void CFastParticleSlamBase::receivedSensorData(SScanLine const& scanline) {
     LOG("=== Update === ");
     LOG("t = " << scanline.translation() << " phi = " << scanline.rotation());
    
    {
        std::vector<std::future<void>> vecfuture;
        boost::for_each(m_vecparticle, [&](auto& p) {
            vecfuture.emplace_back( 
                std::async(std::launch::async,
                    [&] {
                        p.updatePose(scanline);
                    }
                ));
        });
    }

    // 4. Normalize weights (see GridSlamProcessor::normalize())
    {
        // TODO: m_obsSigmaGain
        double const fGain = 1. / ( 3 * /* = m_obsSigmaGain */ m_vecparticle.size());
        auto const fMax = *boost::max_element(boost::adaptors::transform(m_vecparticle, std::mem_fn(&SFastSlamParticle::m_fLogWeight)));
        
        double fWeightSum = 0;
        boost::for_each(m_vecparticle, [&](auto& p) {
            p.m_fWeight = std::exp(fGain * (p.m_fLogWeight - fMax));
            fWeightSum += p.m_fWeight;
        });
        
        m_fNEff = 0;
        boost::for_each(m_vecparticle, [&](auto& p) {
            p.m_fWeight /= fWeightSum;
            m_fNEff += p.m_fWeight*p.m_fWeight;
        });
        m_fNEff = 1.0 / m_fNEff;
    }

    // 5. If neff < threshold, resample
    if(m_fNEff<0.5 * m_vecparticle.size()) {
        LOG("============ Resample ============");
        // Resampling
        // Thrun, Probabilistic robotics, p. 110
        auto const fStepSize = 1.0/m_vecparticle.size();
        auto const r = std::uniform_real_distribution<double>(0.0, fStepSize)(s_rd);
        auto c = m_vecparticle.front().m_fWeight;

        std::vector<int> veciparticle;
        for(int i = 0, m = 0; m<m_vecparticle.size(); ++m) {
            auto const u = r + m * fStepSize;
            while(c<u) {
                ++i;
                c += m_vecparticle[i].m_fWeight;
            }
            veciparticle.emplace_back(i);
            LOG("Keep particle " << i);
        }
        
        std::vector<SFastSlamParticle> vecparticle(m_vecparticle.size());
        auto itparticleOut = vecparticle.begin();
        for(auto itn = veciparticle.begin(); itn!=veciparticle.end(); ++itn) {
            if(boost::next(itn)==veciparticle.end() || *itn!=*boost::next(itn)) {
                *itparticleOut = std::move(m_vecparticle[*itn]);
            } else {
                *itparticleOut = m_vecparticle[*itn];
            }
            ++itparticleOut;
        }
        std::swap(m_vecparticle, vecparticle);
    }
    
    m_itparticleBest = boost::max_element(
        boost::adaptors::transform(m_vecparticle, std::mem_fn(&SFastSlamParticle::m_fWeight))
    ).base();
    m_vecpose.emplace_back(m_itparticleBest->m_pose);

    {
        std::vector<std::future<void>> vecfuture;
        boost::for_each(m_vecparticle, [&](auto& p) {
            vecfuture.emplace_back( 
                std::async(std::launch::async,
                    [&] {
                        p.updateMap(scanline);
                    }
                ));
        });
    }
}

cv::Mat CFastParticleSlamBase::getMap() const {
    ASSERT(m_itparticleBest!=m_vecparticle.end());
    return m_itparticleBest->m_occgrid.ObstacleMapWithPoses(m_vecpose);
}
