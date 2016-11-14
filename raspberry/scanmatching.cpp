#include "scanmatching.h"
#include "robot_configuration.h"
#include "occupancy_grid.inl"

#include "icpPointToPoint.h"
#include <opencv2/imgproc.hpp>

#ifdef ENABLE_LOG
#include <opencv2/imgcodecs/imgcodecs.hpp>     // cv::imread()
#include <iostream>

void DebugOutputScan(cv::Mat const& matObstacle, std::vector<rbt::point<double>> const& vecptfTemplate, char const* szFile) {
    cv::Mat matDebug;
    cv::Mat amatInput[] = {matObstacle, matObstacle, matObstacle};
    cv::merge(amatInput, 3, matDebug);

    boost::for_each(vecptfTemplate, [&](rbt::point<double> const& ptf) {
        rbt::point<int> ptn(ptf);
        auto& vec = matDebug.at<cv::Vec3b>(ptn.y, ptn.x);
        vec.val[0] = 0;
        vec.val[1] = 0;
        vec.val[2] = 255;
    });

    cv::imwrite(szFile, matDebug);
}
#endif

COccupancyGridWithObstacleList::COccupancyGridWithObstacleList() noexcept : m_iEndSorted(0)  
{}

COccupancyGridWithObstacleList::COccupancyGridWithObstacleList(COccupancyGridWithObstacleList const& occgrid) noexcept
    : COccupancyGridBaseT(occgrid)
    , m_vecptfOccupied(occgrid.m_vecptfOccupied)
    , m_iEndSorted(occgrid.m_iEndSorted)
{}

COccupancyGridWithObstacleList::COccupancyGridWithObstacleList(COccupancyGridWithObstacleList&& occgrid) noexcept
    : COccupancyGridBaseT(occgrid)
    , m_vecptfOccupied(std::move(occgrid.m_vecptfOccupied))
    , m_iEndSorted(occgrid.m_iEndSorted)
{}

COccupancyGridWithObstacleList& COccupancyGridWithObstacleList::operator=(COccupancyGridWithObstacleList const& occgrid) noexcept {
    COccupancyGridBaseT::operator=(occgrid);
    m_vecptfOccupied = occgrid.m_vecptfOccupied;
    m_iEndSorted = occgrid.m_iEndSorted;
    return *this;
}

COccupancyGridWithObstacleList& COccupancyGridWithObstacleList::operator=(COccupancyGridWithObstacleList&& occgrid) noexcept {
    COccupancyGridBaseT::operator=(occgrid);
    m_vecptfOccupied = std::move(occgrid.m_vecptfOccupied);
    m_iEndSorted = occgrid.m_iEndSorted;
    return *this;
}

rbt::pose<double> COccupancyGridWithObstacleList::fit(rbt::pose<double> const& poseWorld, SScanLine const& scanline) {
    if(m_iEndSorted<m_vecptfOccupied.size()) {
        auto itptfEndSorted = m_vecptfOccupied.begin()+m_iEndSorted;
        std::sort(itptfEndSorted, m_vecptfOccupied.end());
        m_vecptfOccupied.erase(std::unique(itptfEndSorted, m_vecptfOccupied.end()), m_vecptfOccupied.end());
        std::inplace_merge(m_vecptfOccupied.begin(), itptfEndSorted, m_vecptfOccupied.end());
        m_iEndSorted = m_vecptfOccupied.size();
    }
    
    std::vector<rbt::point<double>> vecptfTemplate;
    scanline.ForEachScan(poseWorld, [&](rbt::pose<double> const& poseScan, double fRadAngle, int nDistance) {
        vecptfTemplate.emplace_back(ToGridCoordinate(Obstacle(poseScan, fRadAngle, nDistance)));
    });
        
#ifdef ENABLE_LOG
    static int c_nCount = 0;
    {
        std::stringstream ss;
        ss << "scanmatch" << c_nCount << "_a.png";
        DebugOutputScan(ObstacleMap(), vecptfTemplate, ss.str().c_str());
    }
    LOG(c_nCount);
    LOG(" Pose estimate = " << poseWorld);
    LOG(" t = " << scanline.translation() << " phi = " << scanline.rotation());
#endif
        
    rbt::pose<double> poseGrid(ToGridCoordinate(poseWorld));
        
    // Transformation matrix from pose
    Matrix R = Matrix::eye(2);
    Matrix t(2,1);
    static_assert(sizeof(rbt::point<double>)==2*sizeof(double), "");
        
    // Use libicp, an iterative closest point implementation (http://www.cvlibs.net/software/libicp/)
    IcpPointToPoint icp(&m_vecptfOccupied[0].x, m_vecptfOccupied.size(), 2);
    icp.fit(&vecptfTemplate[0].x,vecptfTemplate.size(), R, t, 250);
        
    LOG("ICP: R = " << R << " t = " << t);
        
    // Pose from transformation matrix
    Matrix vecLastPose = (R * Matrix(2, 1, &poseGrid.m_pt.x)) + t;
    rbt::pose<double> const poseWorldCorrected(
        ToWorldCoordinate(
            rbt::pose<double>(
                rbt::point<double>(vecLastPose.val[0][0], vecLastPose.val[1][0]),
                poseWorld.m_fYaw - asin(R.val[0][1])
            )
        ));
    
#ifdef ENABLE_LOG
    LOG(" Corrected Pose: " << poseWorldCorrected);
    ++c_nCount;
#endif

    return poseWorldCorrected;
}

void COccupancyGridWithObstacleList::updateGrid(rbt::point<int> const& pt, double fOdds) {
    auto itptfEndSorted =m_vecptfOccupied.begin()+m_iEndSorted;
    rbt::point<double> const ptf(pt);
    auto itpt = std::lower_bound(m_vecptfOccupied.begin(), itptfEndSorted, ptf);
    if(0<fOdds) { // occupied point
        if(itpt==m_vecptfOccupied.end() || *itpt!=ptf) {
            m_vecptfOccupied.emplace_back(pt);
        }
    } else { // free point
        if(itpt!=m_vecptfOccupied.end() && *itpt==ptf) {
            std::swap(*itpt, *boost::prior(itptfEndSorted));
            m_iEndSorted=itpt-m_vecptfOccupied.begin()+1;
        }
    }
}


cv::Mat COccupancyGridWithObstacleList::ObstacleMap() const {
    cv::Mat mat; 
    cv::threshold(m_matfMapLogOdds, mat, 0, 255.0, cv::THRESH_BINARY_INV);
    
    cv::Mat matn; 
    mat.convertTo(matn, CV_8U);
    return matn;
}

cv::Mat COccupancyGridWithObstacleList::ObstacleMapWithPoses(std::vector<rbt::pose<double>> const& vecpose) const {
    return ::ObstacleMapWithPoses(ObstacleMap(), vecpose);
}

CScanMatchingBase::CScanMatchingBase() {
    m_vecpose.emplace_back(rbt::pose<double>::zero());
}

void CScanMatchingBase::receivedSensorData(SScanLine const& scanline) {
    // TODO: Use rotation matrix everywhere
    rbt::pose<double> poseNewCandidate(
        m_vecpose.back().m_pt + scanline.translation().rotated(m_vecpose.back().m_fYaw),
        m_vecpose.back().m_fYaw + scanline.rotation() 
    );
    
    m_vecpose.emplace_back(m_occgrid.fit(poseNewCandidate, scanline));
    scanline.ForEachScan(m_vecpose.back(), [&](rbt::pose<double> const& poseScan, double fRadAngle, int nDistance) {
        m_occgrid.update(poseScan, fRadAngle, nDistance);
    });
}

cv::Mat CScanMatchingBase::getMap() const {
    return m_occgrid.ObstacleMapWithPoses(m_vecpose);
}
