#pragma once

#include "geometry.h"
#include "rover.h"

// The robot microcontroller is reporting each individual Lidar measurement
// We accumulate them in a SScanLine and can then process an entire 180 deg
// scanline at a time.

// The robot moves while the scan line is being accumulated and this is an
// error source. SScanLine::add does consider the change in the robot's 
// position but it can only use the odometry data to do so, which is imprecise.
// In practice, the error depends on how fast the robot moves versus how
// fast the Lidar turns.  
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

    // Process each scan in the scan line
    // 'pose' is the best estimate of the current pose, based 
    // on 'translation()' and 'rotation()'
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

template<typename Base>
struct SAccumulateScanline : Base {
    bool receivedSensorData(SSensorData const& data) {
        if(!this->m_scanline.add(data)) {
            Base::receivedSensorData(this->m_scanline);
            this->m_scanline.clear();
            this->m_scanline.add(data);
            return true;
        }
        return false;
    }
private:
    SScanLine m_scanline; // accumulates individual lidar scans
}; 