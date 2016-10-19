#pragma once

#include "geometry.h"
#include "rover.h"

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