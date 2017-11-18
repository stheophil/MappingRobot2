#include "scanmatching.h"
#include "robot_configuration.h"
#include "occupancy_grid.inl"

#include "icpPointToPoint.h"
#include <opencv2/imgproc.hpp>

// #define ENABLE_SCANMATCH_LOG

#include <opencv2/imgcodecs/imgcodecs.hpp>     // cv::imread()

#ifdef ENABLE_SCANMATCH_LOG
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
    : COccupancyGridBaseT(std::move(occgrid))
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
    COccupancyGridBaseT::operator=(std::move(occgrid));
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
    
    if(m_vecptfOccupied.size()<10) return poseWorld;
    
    std::vector<rbt::point<double>> vecptfTemplate;
    boost::for_each(scanline.m_vecscan, [&](auto const& scan) {
        vecptfTemplate.emplace_back(ToGridCoordinate(Obstacle(poseWorld, scan.m_fRadAngle, scan.m_nDistance)));
    });
        
#ifdef ENABLE_SCANMATCH_LOG
    static int c_nCount = 0;
    {
        std::stringstream ss;
        ss << "scanmatch" << c_nCount << "_a.png";
        DebugOutputScan(ObstacleMap(), vecptfTemplate, ss.str().c_str());
    }
    LOG(c_nCount);
#endif
        
    rbt::pose<double> poseGrid(ToGridCoordinate(poseWorld));
        
    // Transformation matrix from pose
    Matrix R = Matrix::eye(2);
    Matrix t(2,1);
    static_assert(sizeof(rbt::point<double>)==2*sizeof(double), "");
        
    // Use libicp, an iterative closest point implementation (http://www.cvlibs.net/software/libicp/)
    IcpPointToPoint icp(&m_vecptfOccupied[0].x, m_vecptfOccupied.size(), 2);
    icp.fit(&vecptfTemplate[0].x,vecptfTemplate.size(), R, t, 250);
    
#ifdef ENABLE_SCANMATCH_LOG
    LOG("ICP: R = " << R << " t = " << t);
#endif

    // Pose from transformation matrix
    Matrix vecLastPose = (R * Matrix(2, 1, &poseGrid.m_pt.x)) + t;
    rbt::pose<double> const poseWorldCorrected(
        ToWorldCoordinate(
            rbt::pose<double>(
                rbt::point<double>(vecLastPose.val[0][0], vecLastPose.val[1][0]),
                poseWorld.m_fYaw - asin(R.val[0][1])
            )
        ));
    
#ifdef ENABLE_SCANMATCH_LOG
    LOG(" Corrected Pose: " << poseWorldCorrected);
     {
     
        std::vector<rbt::point<double>> vecptfTemplateCorrected;
        boost::for_each(scanline.m_vecscan, [&](auto const& scan) {
            vecptfTemplateCorrected.emplace_back(ToGridCoordinate(Obstacle(poseWorldCorrected, scan.m_fRadAngle, scan.m_nDistance)));
        });

        std::stringstream ss;
        ss << "scanmatch" << c_nCount << "_b.png";
        DebugOutputScan(ObstacleMap(), vecptfTemplateCorrected, ss.str().c_str());
    }
    ++c_nCount;
#endif

    return poseWorldCorrected;
}

void COccupancyGridWithObstacleList::updateGrid(rbt::point<int> const& pt, double fOdds) {
    auto itptfEndSorted =m_vecptfOccupied.begin()+m_iEndSorted;
    rbt::point<double> const ptf(pt);
    auto itpt = std::lower_bound(m_vecptfOccupied.begin(), itptfEndSorted, ptf);
    if(c_fFreeThreshold<fOdds) { // occupied point
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
    cv::Mat matnMapLogOdds;
    m_matfMapLogOdds.convertTo(matnMapLogOdds, CV_8U, /*alpha*/ -1, 128);
    // p = 1/(1 + exp(fOdds))
    // So fOdds = 0 means p = 0.5, less means probably free space, higher means probably occupied

    // In matnMapLogOdds, each pixel is -1 * fOdds + 128, so p = 0.5 is color 128, 
    // less means probably occupied, higher means probably free. 
    
    cv::Mat matn; 
    // Set everything to 0 that is < 127, leave rest as is
    cv::threshold(matnMapLogOdds, matn, 128-c_fFreeThreshold-1, /*ignored*/ 255, cv::THRESH_TOZERO);

    cv::Mat matnFree; 
    // Set everything to 255 that is > 129, everything else to 0
    cv::threshold(matnMapLogOdds, matnFree, 128+c_fFreeThreshold, 255.0, cv::THRESH_BINARY);
    
    matnFree.copyTo(matn, /*mask*/ matnFree);
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
    boost::for_each(scanline.m_vecscan, [&](auto const& scan) {
        m_occgrid.update(m_vecpose.back(), scan.m_fRadAngle, scan.m_nDistance);
    });
}

cv::Mat CScanMatchingBase::getMap() const {
    return m_occgrid.ObstacleMapWithPoses(m_vecpose);
}
