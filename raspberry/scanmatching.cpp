#include "scanmatching.h"
#include "robot_configuration.h"

#include "icpPointToPoint.h"
#include <opencv2/imgproc.hpp>

#ifdef ENABLE_LOG
#include <opencv2/imgcodecs/imgcodecs.hpp>     // cv::imread()
#include <iostream>

void DebugOutputScan(cv::Mat const& matObstacle, std::vector<double> const& vecptfTemplate) {
    cv::Mat matDebug;
    cv::Mat amatInput[] = {matObstacle, matObstacle, matObstacle};
    cv::merge(amatInput, 3, matDebug);

    boost::for_each(vecptfTemplate, [&](rbt::point<double> const& ptf) {
        auto ptn = ToGridCoordinate(ptf);
        auto& vec = matDebug.at<cv::Vec3b>(ptn.y, ptn.x);
        vec.val[0] = 0;
        vec.val[1] = 0;
        vec.val[2] = 255;
    });

    std::stringstream ss;
    ss << "scanmatch" << c_nCount << "_b.png";
    cv::imwrite(ss.str().c_str(), matDebug);
}
#endif

CScanMatchingBase::CScanMatchingBase() {
    m_vecpose.emplace_back(rbt::pose<double>::zero());
}

void CScanMatchingBase::receivedSensorData(SScanLine const& scanline) {
    // TODO: Cache this too?
    // TODO: Use rotation matrix everywhere
    std::vector<double> vecfModel;
    for(auto it=m_occgrid.ObstacleMap().begin<std::uint8_t>(); it != m_occgrid.ObstacleMap().end<std::uint8_t>(); ++it) {
        if(0==*it) {
            vecfModel.emplace_back(it.pos().x);
            vecfModel.emplace_back(it.pos().y);
        }
    }
    
    // Use libicp, an iterative closest point implementation (http://www.cvlibs.net/software/libicp/)
    IcpPointToPoint icp(&vecfModel[0], vecfModel.size()/2, 2);

    rbt::pose<double> poseNewCandidate(
        m_vecpose.back().m_pt + scanline.translation().rotated(m_vecpose.back().m_fYaw),
        m_vecpose.back().m_fYaw + scanline.rotation() 
    );

    std::vector<rbt::point<double>> vecptfTemplate;    
    scanline.ForEachScan(poseNewCandidate, [&](rbt::pose<double> const& poseScan, double fRadAngle, int nDistance) {
        vecptfTemplate.emplace_back(ToGridCoordinate(Obstacle(poseScan, fRadAngle, nDistance)));
    });

#ifdef ENABLE_LOG
    static int c_nCount = 0;
    {
        std::stringstream ss;
        ss << "scanmatch" << c_nCount << "_a.png";
        DebugOutputScan(m_occgrid.ObstacleMap(), vecptfTemplate, ss.str().c_str());
    }
#endif 

    LOG(c_nCount << " Old Pose: " << m_vecpose.back() << " New Pose: " << poseNewCandidate);
    LOG(" t = " << scanline.translation() << " phi = " << scanline.rotation());

    Matrix R = Matrix::eye(2);
    Matrix t(2,1);
    
    static_assert(sizeof(rbt::point<double>)==2*sizeof(double), "");
    icp.fit(&vecptfTemplate[0].x,vecptfTemplate.size(), R, t, 200);
    
    LOG("ICP: R = " << R << " t = " << t);

    auto const ptfGrid = rbt::point<double>(ToGridCoordinate(poseNewCandidate.m_pt));
    Matrix vecLastPose = (R * Matrix(2, 1, &ptfGrid.x)) + t;
    m_vecpose.emplace_back(
        rbt::point<double>(ToWorldCoordinate(rbt::point<int>(vecLastPose.val[0][0], vecLastPose.val[1][0]))),
        poseNewCandidate.m_fYaw - asin(R.val[0][1])
    );

    LOG(" Corrected Pose: " << m_vecpose.back());

    boost::for_each(vecptfTemplate, [&](rbt::point<double>& ptf) {
        auto vec = (R * Matrix(2, 1, &ptf.x)) + t;
        ptf = rbt::point<double>(ToWorldCoordinate(rbt::point<int>(vec.val[0][0], vec.val[1][0])));
    });

#ifdef ENABLE_LOG
    {
        std::stringstream ss;
        ss << "scanmatch" << c_nCount << "_b.png";
        DebugOutputScan(m_occgrid.ObstacleMap(), vecptfTemplate, ss.str().c_str());
        ++c_nCount;
    }
#endif

    m_occgrid.update(
        m_vecpose.back(),
        vecptfTemplate
    );

}

cv::Mat CScanMatchingBase::getMap() const {
    return m_occgrid.ObstacleMapWithPoses(m_vecpose);
}
