#include "scanline.h"
#include "robot_configuration.h"

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
