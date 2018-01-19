#include "path_finding.h"

#include "error_handling.h"
#include "rover.h"
#include "geometry.h"
#include "robot_configuration.h"

#include <opencv2/opencv.hpp>

#include <vector>
#include <queue>

#include <boost/optional.hpp>
// A* path finding algorithm
template<typename TNode, typename FMinimalNodeCost, typename FForEachNeighbor>
boost::optional<TNode> GenericAStar(rbt::pose<double> const& posefStart, 
    rbt::point<double> const& ptfEnd, 
    FMinimalNodeCost MinimalNodeCost, 
    FForEachNeighbor ForEachNeighbor
) {
    auto const posenStart = ToGridCoordinate(posefStart);
    auto const ptnEnd = ToGridCoordinate(ptfEnd);

    auto GreaterDistance = [&](TNode const& lhs, TNode const& rhs) noexcept {
        return (lhs.m_pt - ptnEnd).Abs() + lhs.m_fCost > (rhs.m_pt - ptnEnd).Abs() + rhs.m_fCost;
    };

    std::priority_queue<TNode, std::vector<TNode>, decltype(GreaterDistance)> queue(GreaterDistance);
    
    TNode nodeStart(posenStart);
    queue.push(nodeStart);
    MinimalNodeCost(nodeStart) = nodeStart.m_fCost;

    while(!queue.empty()) {
        auto const nodeTop = queue.top();
        queue.pop();
        if(MinimalNodeCost(nodeTop) < nodeTop.m_fCost) continue;

        if(nodeTop.m_pt==ptnEnd) return nodeTop;

        ForEachNeighbor(
            nodeTop, 
            [&](TNode const& nodeNeighbor) {
                if(rbt::assign_min(MinimalNodeCost(nodeNeighbor), nodeNeighbor.m_fCost)) {
                    queue.push(nodeNeighbor);
                }
            });
    }

    return boost::none;
}

std::vector<rbt::point<double>> FindPath(cv::Mat matn, rbt::pose<double> const& posefStart, rbt::point<double> const& ptfEnd) {
    cv::Mat matnEroded;
    auto const nMaxExtent = std::max(c_nRobotWidth/c_nScale, c_nRobotHeight/c_nScale);
    cv::erode(
        matn, 
        matnEroded, 
        cv::Mat::ones(nMaxExtent, nMaxExtent, CV_8U)
    );
    
    // Include costs of traveling close to an obstacle in calculation
    cv::Mat matnGauss;
    auto const nMaxExtentOdd = 2*nMaxExtent + 1;
    cv::GaussianBlur(matnEroded, matnGauss, cv::Size(nMaxExtentOdd, nMaxExtentOdd), 0, 0);

    float aanMinimalCost[c_nMapExtent][c_nMapExtent];
    std::fill_n(&aanMinimalCost[0][0], c_nMapExtent*c_nMapExtent, std::numeric_limits<float>::max());

    struct node {
        node() {}
        node(rbt::pose<int> const& pose) : m_pt(pose.m_pt), m_fCost(0) {}
        node(rbt::point<int> const& pt, float fCost) : m_pt(pt), m_fCost(fCost) {}
        rbt::point<int> m_pt; // in grid coordinates
        float m_fCost;
    };

    auto MinimalNodeCost = [&](node const& node) noexcept -> float& {
        return aanMinimalCost[node.m_pt.x][node.m_pt.y];
    };

    auto ForEachNeighbor = [&](node const& n, auto fn) noexcept {
        for(int x = -1; x <= 1; ++x) {
            for(int y = -1; y <= 1; ++y) {
                auto const ptNext = n.m_pt + rbt::size<int>(x, y);
                if((0!=x || 0!=y)
                && 128<matnGauss.at<std::uint8_t>(ptNext.y, ptNext.x)) {
                    fn(node(
                        ptNext,
                        n.m_fCost + (0==x || 0==y ? 1.0f : M_SQRT2) * (1 + (255 - matnGauss.at<std::uint8_t>(ptNext.y, ptNext.x))/10)
                    ));
                }
                }
    }
    };

    std::vector<rbt::point<double>> vecptfResult;
    if(auto onode = GenericAStar<node>(posefStart, ptfEnd, MinimalNodeCost, ForEachNeighbor)) {
    vecptfResult.emplace_back(ptfEnd);
        auto const ptnStart = ToGridCoordinate(posefStart.m_pt);
        for(auto nodePrev = *onode; nodePrev.m_pt!=ptnStart; ) {
        float fMinCost = std::numeric_limits<float>::max();
            node nodeMin;
            ForEachNeighbor(
                nodePrev,
                [&](node const& node) {
                    if(rbt::assign_min(fMinCost, MinimalNodeCost(node))) {
                        nodeMin = node;
                }
            }
        );

            nodePrev = nodeMin;
            vecptfResult.emplace_back(ToWorldCoordinate(rbt::point<double>(nodeMin.m_pt)));
        }
    }
    return vecptfResult;
}
