#pragma once

#include "geometry.h"
#include "rover.h"

// The XV11 Neato Lidar is reporting 4 Lidar measurements at a time.
// We accumulate them in a SScanLine and can then process an entire 360 deg
// scanline at a time.

// The robot moves while the scan line is being accumulated and this is an
// error source. With the relatively fast XV11 Neato Lidar, we ignore this error.
struct SScanLine {
    struct SScan {
        SScan(int nAngle, int nDistance)
            : m_nAngle(nAngle), m_fRadAngle(rbt::rad(nAngle)), m_nDistance(nDistance)
        {}

        int m_nAngle; // in degrees
        double m_fRadAngle; // in radians
        int m_nDistance;
    };

    rbt::pose<double> m_pose = rbt::pose<double>::zero();
    std::vector< SScan > m_vecscan;

    rbt::size<double> translation() const;
    double rotation() const;
    
    // returns false iff scan line is complete, i.e., contains 360 degree measurements
    // and data belongs into new SScanLine
    void add(SLidarData const& data);
    void add(SOdometryData const& odom); 
    void clear();
};

template<typename Func>
void ForEachScan(SLidarData const& lidar, Func fn) {
    int nAngle = (lidar.m_nIndex - c_nFIRST_LIDAR_INDEX) * 4;
    for(auto it = boost::begin(lidar.m_adata); it!=boost::end(lidar.m_adata); ++it) {
        if(!it->m_flagInvalidData) {
            // 180 degrees is in front of robot, lidar rotates ccw
            fn(SScanLine::SScan(nAngle<180 ? nAngle + 180 : nAngle - 180, it->m_nDistance/10)); // SLidarData reports distance is in mm
        }
        ++nAngle;
    }
}