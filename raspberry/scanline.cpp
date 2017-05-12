#include "scanline.h"
#include "robot_configuration.h"

/////////////////////
// SScanLine

bool SScanLine::add(SLidarData const& lidar) {
    return ForEachAngleDistance(lidar, [&](int nAngle, int nDistance) {
		return add(nAngle, nDistance);
    });
}

bool SScanLine::add(int nAngle, int nDistance) {
    if(!m_vecscan.empty() && m_vecscan.front().m_nAngle==nAngle) return false;
    m_vecscan.emplace_back(nAngle, nDistance);
    return true;
}

void SScanLine::add(SOdometryData const& odom) {
    m_pose = UpdatePose(m_pose, odom);
}

rbt::size<double> SScanLine::translation() const {
    return rbt::size<double>(m_pose.m_pt);
}

double SScanLine::rotation() const {
    return m_pose.m_fYaw;
}

void SScanLine::clear() {
    m_vecscan.clear();
}
