#include "scanline.h"
#include "robot_configuration.h"

bool SLidarData::ValidChecksum() const {
    int nChecksum = 0;

    static_assert(sizeof(SLidarData)==22, "");
    static_assert(sizeof(m_nChecksum)==2, "");
    for(unsigned short const* pshort = reinterpret_cast<unsigned short const*>(this);
        pshort < reinterpret_cast<unsigned short const*>(this)+10;
        ++pshort) 
    {
        nChecksum = (nChecksum << 1) + *pshort;
    }
    nChecksum = (nChecksum & 0x7FFF) + (nChecksum >> 15);
    nChecksum &= 0x7FFF;

    return nChecksum == m_nChecksum;
}
/////////////////////
// SScanLine
void SScanLine::add(SLidarData const& lidar) {
    ForEachScan(lidar, [&](SScanLine::SScan const& scan) {
		m_vecscan.emplace_back(scan);
    });
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
    m_pose = rbt::pose<double>::zero();
    m_vecscan.clear();
}
