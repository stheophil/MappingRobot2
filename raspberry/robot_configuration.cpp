#include "rover.h"
#include "geometry.h"
#include "particle_slam.h"
#include "error_handling.h"
#include "robot_configuration.h"

#include <random>
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

    double gauss_probability(double fX, double fS) { // median = 0
        return std::exp(-1 * std::pow(fX, 2) / (2*std::pow(fS, 2)))
            / (std::sqrt(2*M_PI) * fS);
    }
}

double InitialYaw(SSensorData const& sensordata) {
    return 0; // ignore compass entirely for now
}

rbt::pose<double> UpdatePose(rbt::pose<double> const& pose, SSensorData const& sensordata) {
    // differential drive: http://planning.cs.uiuc.edu/node659.html
    // take average of two wheels on each side
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

    return rbt::pose<double>(
        pose.m_pt + sz,
        pose.m_fYaw + fDeltaYaw
    );
} 

rbt::point<int> ToGridCoordinate(rbt::point<double> const& pt) {
    return rbt::point<int>(pt/c_nScale) + rbt::size<int>(c_nMapExtent, c_nMapExtent)/2;
}

rbt::pose<int> ToGridCoordinate(rbt::pose<double> const& pose) {
    return rbt::pose<int>( ToGridCoordinate(pose.m_pt), pose.m_fYaw );
}

rbt::point<double> Obstacle(rbt::pose<double> const& pose, double fRadAngle, double fDistance) {
    auto const szfLidar = rbt::size<double>::fromAngleAndDistance(fRadAngle, fDistance)
        + c_szfLidarOffset;
    return rbt::point<double>(pose.m_pt + szfLidar.rotated(pose.m_fYaw));
}

static std::random_device s_rd;
rbt::pose<double> sample_motion_model(rbt::pose<double> const& pose, rbt::size<double> const& szf, double fRadAngle) {
    // http://gki.informatik.uni-freiburg.de/lehre/ws0203/Robotik/papers/kalman/kurt_robot_notes.pdf
    // TODO: Make measurements to get actual errors
    double const c_fRangeStdDev = 0.1; // 0.1 cm/cm range error 
    double const c_fTurnVar = 0.09; // (0.3 rad)^2 / rad turn error
    double const c_fDriftVar = 0.0003; // (pi/18 rad)^2/100 cm drift error ~ (5 deg)^2/m 

    auto const fDistance = szf.Abs();
    rbt::size<double> const szfTranslation = [&] {
        if(0!=fDistance) { 
            // there was actual movement -> sample range error
            auto const szfSampled = szf.normalized() 
                * std::normal_distribution<double>(fDistance, c_fRangeStdDev * fDistance)(s_rd);
            return szfSampled.rotated(pose.m_fYaw);
        } else {
            return rbt::size<double>::zero();
        }
    }();

    return rbt::pose<double>(
        pose.m_pt + szfTranslation,
        pose.m_fYaw + 
            // sample from rotation errors
            std::normal_distribution<double>(
                fRadAngle, 
                std::sqrt(c_fTurnVar * std::abs(fRadAngle) + c_fDriftVar * fDistance)
            )(s_rd)
        );
}

double measurement_model_map(rbt::pose<double> const& pose, 
    SScanLine const& scanline, 
    std::function<double (rbt::point<double>)> Distance
) {
    // Likelihood field model
    // Thrun, Probabilistic Robotics, p. 169ff

    // TODO: Learn sensor parameters? 
    // We do indoor-scanning, so max-distance reading is ignored
    double const z_hit = 0.9;
    double const z_rand = 0.1;

    double const c_fSensorSigma = 2; // ~ +-10cm with current map scale, in grid coordinates

    double fWeight = 1.0;
    scanline.ForEachScan(pose, [&](rbt::pose<double> const& poseScan, double fAngle, int nDistance) {
        double const fLikelihood = 
            z_hit * gauss_probability( Distance(Obstacle(poseScan, fAngle, nDistance)), c_fSensorSigma) 
            + z_rand; 
        fWeight = fWeight * fLikelihood;
    });
    return fWeight;
}
