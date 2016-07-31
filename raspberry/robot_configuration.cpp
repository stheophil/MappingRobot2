#include "rover.h"
#include "geometry.h"

#include <opencv2/imgproc.hpp>

namespace {
    int const c_nWheelRadius = 6; // cm
    double const c_fWheelDistance = 24.5; // cm

    // Offset of robot center to lidar center as x, y coordinates
    // y-axis points into direction of robot front
    rbt::size<double> const c_szfLidarOffset(0, 7); 

    double encoderTicksToRadians(short nTicks) { // Note: Formula depends on wheel encoders
        return nTicks * 6.0 * M_PI / 1000.0;
    }

    double rad(double fAngle) {
        return fAngle * M_PI / 180;
    }
}

double InitialYaw(SSensorData const& sensordata) {
    return 0; // ignore compass entirely for now
}

rbt::pose UpdatePose(rbt::pose const& pose, SSensorData const& sensordata) {
    // differential drive: http://planning.cs.uiuc.edu/node659.html
    // take average of left wheel movement & right wheel movement

    auto const fRadLeft = encoderTicksToRadians((sensordata.m_anEncoderTicks[0] + sensordata.m_anEncoderTicks[2])/2);
    auto const fRadRight = encoderTicksToRadians((sensordata.m_anEncoderTicks[1] + sensordata.m_anEncoderTicks[3])/2);

    auto const fT = (fRadRight + fRadLeft)/2;
    auto const fR = fRadRight - fRadLeft;

    rbt::size<double> const sz(
        c_nWheelRadius * fT * std::cos(pose.m_fYaw),
        c_nWheelRadius * fT * std::sin(pose.m_fYaw)
    );

    // fDeltaYaw is counter-clockwise
    auto const fDeltaYaw = c_nWheelRadius * fR / c_fWheelDistance;

    return rbt::pose(
        pose.m_pt + sz,
        pose.m_fYaw + fDeltaYaw
    );
} 

void ForEachCell(rbt::point<int> const& ptnGrid, double fYaw, SSensorData const& sensordata, 
    cv::Mat const& matGrid, int nScale, 
    std::function<void(rbt::point<int> const&, float)> UpdateGrid  
) {
    auto const szfLidar = rbt::size<double>::fromAngleAndDistance(rad(sensordata.m_nAngle), sensordata.m_nDistance)
        + c_szfLidarOffset;
    
    rbt::size<int> sznLidarInRobotAngle(szfLidar.rotated(fYaw)/nScale);
    rbt::point<int> const ptnLidar(ptnGrid + sznLidarInRobotAngle);

    cv::LineIterator itpt(matGrid, ptnGrid, ptnLidar);
    for(int i = 0; i < itpt.count; i++, ++itpt) {
        UpdateGrid(rbt::point<int>(itpt.pos()),  i<itpt.count-1 
            ? -0.5 // free
            : (100.0 / nScale) // occupied  
        );
    }
}