#include "path_finding.h"

#include "error_handling.h"
#include "rover.h"
#include "geometry.h"
#include "robot_configuration.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>

// A* path finding algorithm
std::vector<rbt::point<double>> FindPath(cv::Mat matn, rbt::point<double> const& ptfStart, rbt::point<double> const& ptfEnd) {
    cv::Mat matnEroded;
    auto const nMaxExtent = std::max(c_nRobotWidth/c_nScale, c_nRobotHeight/c_nScale);
    cv::erode(
        matn, 
        matnEroded, 
        cv::Mat::ones(nMaxExtent, nMaxExtent, CV_8U)
    );

    // TODO: Include costs of traveling close to an obstacle in calculation
    // cv::Mat matnGauss;
    // auto const nMaxExtentOdd = nMaxExtent + 1 - nMaxExtent%2;
    // cv::GaussianBlur(matnEroded, matnGauss, cv::Size(nMaxExtentOdd, nMaxExtentOdd), 0, 0);

    float aanMinimalCost[c_nMapExtent][c_nMapExtent];
    std::fill_n(&aanMinimalCost[0][0], c_nMapExtent*c_nMapExtent, std::numeric_limits<float>::max());

    auto MinimalNodeCost = [&](rbt::point<int> const& pt) noexcept -> float& {
        return aanMinimalCost[pt.x][pt.y];
    };

    auto ForEachNeighborCoordinate = [](rbt::point<int> const& pt, auto fn) noexcept {
        for(int x = -1; x <= 1; ++x) {
            for(int y = -1; y <= 1; ++y) {
                if(0!=x || 0!=y) {                    
                    fn(pt + rbt::size<int>(x, y), 0==x || 0==y ? 1.0f : M_SQRT2);
                }
            }
        }
    };

    auto const ptnStart = ToGridCoordinate(ptfStart);
    auto const ptnEnd = ToGridCoordinate(ptfEnd);

    struct node {
        rbt::point<int> m_pt; // in grid coordinates
        float m_fCost;
    };

    auto LessSqrDistance = [&](node const& lhs, node const& rhs) noexcept {
        return (lhs.m_pt - ptnEnd).SqrAbs() < (rhs.m_pt - ptnEnd).SqrAbs();
    };

    std::priority_queue<node, std::vector<node>, decltype(LessSqrDistance)> queue(LessSqrDistance);
    queue.push(node{ptnStart, 0});
    
    while(!queue.empty()) {
        auto const nodeTop = queue.top();
        queue.pop();

        if(rbt::assign_min(MinimalNodeCost(nodeTop.m_pt), nodeTop.m_fCost)) {
            if(nodeTop.m_pt==ptnEnd) break;

            ForEachNeighborCoordinate(
                nodeTop.m_pt, 
                [&](rbt::point<int> const& pt, float fDeltaCost) {
                    node nodeNeighbor{pt, nodeTop.m_fCost + fDeltaCost};

                    if(nodeNeighbor.m_fCost < MinimalNodeCost(nodeNeighbor.m_pt)
                    && 128<matnEroded.at<std::uint8_t>(pt.y, pt.x)) {
                        queue.push(nodeNeighbor);
                    }
                });
        }
    }

    std::vector<rbt::point<double>> vecptfResult;
    vecptfResult.emplace_back(ptfEnd);

    for(auto ptnPrev = ptnEnd; ptnPrev!=ptnStart; ) {
        float fMinCost = std::numeric_limits<float>::max();
        rbt::point<int> ptnMin;
        ForEachNeighborCoordinate(
            ptnPrev,
            [&](rbt::point<int> const& pt, float /*fDeltaCost*/) {
                if(rbt::assign_min(fMinCost, MinimalNodeCost(pt))) {
                    ptnMin = pt;
                }
            }
        );

        ptnPrev = ptnMin;
        vecptfResult.emplace_back(ToWorldCoordinate(rbt::point<double>(ptnMin)));
    }
    return vecptfResult;
}
