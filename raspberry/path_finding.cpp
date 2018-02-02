#include "path_finding.h"

#include "error_handling.h"
#include "rover.h"
#include "geometry.h"
#include "robot_configuration.h"

#include <opencv2/opencv.hpp>

#include <vector>
#include <queue>

#include <boost/functional/hash.hpp>
#include <boost/optional.hpp>
#include <boost/range/algorithm/reverse.hpp>

// A* path finding algorithm
template<typename TNode, typename T, typename FMinimalNodeCost, typename FForEachNeighbor, typename FIsGoal>
boost::optional<TNode> GenericAStar(rbt::pose<T> const& poseStart, 
    rbt::point<T> const& ptEnd, 
    FMinimalNodeCost MinimalNodeCost, 
    FForEachNeighbor ForEachNeighbor,
	FIsGoal IsGoal
) {
    auto GreaterDistance = [&](TNode const& lhs, TNode const& rhs) noexcept {
        return (lhs.Position() - ptEnd).Abs() + lhs.m_fCost > (rhs.Position() - ptEnd).Abs() + rhs.m_fCost;
    };

    std::priority_queue<TNode, std::vector<TNode>, decltype(GreaterDistance)> queue(GreaterDistance);
    
    TNode nodeStart(poseStart);
    queue.push(nodeStart);
    MinimalNodeCost(nodeStart) = nodeStart.m_fCost;

    while(!queue.empty()) {
        auto const nodeTop = queue.top();
        queue.pop();

        if(MinimalNodeCost(nodeTop) < nodeTop.m_fCost) continue;

        if(IsGoal(nodeTop, ptEnd)) return nodeTop;

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

		rbt::point<int> Position() const { return m_pt; }

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
	auto IsGoal = [](node const& n, rbt::point<int> const& ptnEnd) {
		return n.m_pt == ptnEnd;
	};

	auto const posenStart = ToGridCoordinate(posefStart);

    std::vector<rbt::point<double>> vecptfResult;
    if(auto onode = GenericAStar<node>(posenStart, ToGridCoordinate(ptfEnd), MinimalNodeCost, ForEachNeighbor, IsGoal)) {
        vecptfResult.emplace_back(ptfEnd);
        auto const ptnStart = posenStart.m_pt;
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


namespace {
    struct config_space_node {
		config_space_node() {}
        config_space_node(rbt::pose<double> const& pose) : m_pose(pose) {}
        
		rbt::point<double> Position() const { return m_pose.m_pt; }
		
		int RoundedYaw() const { return (int)std::round(m_pose.m_fYaw*100); }
		rbt::point<int> RoundedPosition() const { 
			return rbt::point<int>((int)std::round(m_pose.m_pt.x), (int)std::round(m_pose.m_pt.y));
		}
        rbt::pose<double> m_pose;

		// as used for robot commands, encoder ticks / s?
        int m_nSpeedLeft = 0;
        int m_nSpeedRight = 0;

        // path cost
        float m_fCost = 0;

		config_space_node const* m_pnodeParent = nullptr;

        friend bool operator==(config_space_node const& lhs, config_space_node const& rhs) {
            return lhs.RoundedPosition()==rhs.RoundedPosition()
				&& lhs.RoundedYaw()==rhs.RoundedYaw()
                && lhs.m_nSpeedLeft==rhs.m_nSpeedLeft
                && rhs.m_nSpeedRight==rhs.m_nSpeedRight;
        }
    };

	constexpr int c_nSpeedStep = 100;
	constexpr double c_fTimeStep = 0.2; // s
	const float c_fHighTravelDistance = encoderTicksToCm(c_nMaxSpeed*0.8*c_fTimeStep);
}

namespace std {
	template <> struct hash<config_space_node> {
		size_t operator()(config_space_node const& node) const {
			std::size_t seed = 0;
			auto pt = node.RoundedPosition();
			boost::hash_combine(seed, pt.x);
			boost::hash_combine(seed, pt.y);
			boost::hash_combine(seed, node.RoundedYaw());
			boost::hash_combine(seed, node.m_nSpeedLeft);
			boost::hash_combine(seed, node.m_nSpeedRight);
			return seed;
		}
	};
}

std::vector<rbt::pose<double>> PathConfigurationSpace(cv::Mat matn, rbt::pose<double> const& posefStart, rbt::point<double> const& ptfEnd) {
    auto const vecptf = FindPath(matn, posefStart, ptfEnd);

	// TODO: Share code?
	cv::Mat matnEroded;
    auto const nMaxExtent = std::max(c_nRobotWidth/c_nScale, c_nRobotHeight/c_nScale);
    cv::erode(
        matn, 
        matnEroded, 
        cv::Mat::ones(nMaxExtent, nMaxExtent, CV_8U)
    );
    
    cv::Mat matnGauss;
    auto const nMaxExtentOdd = 2*nMaxExtent + 1;
    cv::GaussianBlur(matnEroded, matnGauss, cv::Size(nMaxExtentOdd, nMaxExtentOdd), 0, 0);

	cv::Mat matnPath = cv::Mat::zeros(matn.size(), CV_8U);
	rbt::point<int> ptnPrev = ToGridCoordinate(vecptf.front());
	boost::for_each(vecptf, [&](rbt::point<double> const& ptf) {
		auto const ptnGrid = ToGridCoordinate(ptf);
        cv::line(matnPath, ptnPrev, ptnGrid, cv::Scalar(255), 50/c_nScale);
        ptnPrev = ptnGrid;
	});

	int cExpanded = 0;
	std::unordered_map<config_space_node, float> mapnodefCosts;
	auto onode = GenericAStar<config_space_node>(
		posefStart, 
		// Calculating a longer path in configuration space takes too much time to compute due to large state space.
		*(vecptf.end() - 80), 
		[&](config_space_node const& node) noexcept -> float& {
			return mapnodefCosts.emplace(node, std::numeric_limits<float>::max()).first->second;
		},
		[&](config_space_node const& node, auto fn) noexcept {
			++cExpanded;

			auto const pnodeParent = [&] {
				auto const itpair = mapnodefCosts.find(node);
				ASSERT(itpair!=mapnodefCosts.end());
				return &itpair->first;
			}();

			for(int nStepLeft = -1; nStepLeft <= 1; ++nStepLeft) {
				// [decelerate both, left, right, no change, accelerate right, left, both]
				for(int nStepRight = -1; nStepRight <= 1; ++nStepRight) {
					// -> Calculate new positions / angle 
					auto nodeNeighbor = node;
					nodeNeighbor.m_pnodeParent = pnodeParent;
					nodeNeighbor.m_nSpeedLeft += nStepLeft * c_nSpeedStep;
					nodeNeighbor.m_nSpeedRight += nStepRight * c_nSpeedStep;

					if(std::abs(nodeNeighbor.m_nSpeedLeft)<=c_nMaxSpeed
					&& std::abs(nodeNeighbor.m_nSpeedRight)<=c_nMaxSpeed
					&& std::abs(nodeNeighbor.m_nSpeedRight-nodeNeighbor.m_nSpeedLeft)<=400) 
					{
						nodeNeighbor.m_pose = UpdatePose(
							node.m_pose, 
							nodeNeighbor.m_nSpeedLeft*c_fTimeStep, 
							nodeNeighbor.m_nSpeedRight*c_fTimeStep
						);

						cv::LineIterator itpt(
							matn, 
							ToGridCoordinate(node.Position()), 
							ToGridCoordinate(nodeNeighbor.Position())
						);

						float fWeightedCost = 0;
						for(int i = 0; i < itpt.count; ++i, ++itpt) {    	
							auto const pt = rbt::point<int>(itpt.pos());
							// -> check if still in range of shortest path 
							if(matnPath.at<std::uint8_t>(pt.y, pt.x)<255) {
								goto outside_range;
							}

							// integrate costs over node.m_pt -> nodeNeighbor.m_pt
							fWeightedCost += std::pow((255.0 - matnGauss.at<std::uint8_t>(pt.y, pt.x))/30, 2);
						}
						{
							fWeightedCost = std::max(1.f, fWeightedCost/itpt.count);
							float const fDistance = (nodeNeighbor.Position()-node.Position()).Abs();
							// Penalize several short moves, i.e., slow moves
							nodeNeighbor.m_fCost += fDistance * std::max(1.0, std::pow(c_fHighTravelDistance/fDistance, 2)) * fWeightedCost;
						}

						fn(nodeNeighbor);

outside_range: 
						;
					}
				}
			}
		},
		[](config_space_node const& node, rbt::point<double> const ptfEnd) {
			return (node.m_pose.m_pt - ptfEnd).SqrAbs() < c_nScale*c_nScale;
		}
	);

	std::cout << "Discovered " << mapnodefCosts.size() << " states, " << cExpanded << " expanded." << std::endl;
	std::vector<rbt::pose<double>> vecposef;
	if(onode) {	
		auto const* pnodeNext = &*onode;
		while(pnodeNext) {
			std::cout << pnodeNext->m_pose << 
			" (" << pnodeNext->m_nSpeedLeft << ", " << pnodeNext->m_nSpeedRight << ") " << std::endl;

			vecposef.emplace_back(pnodeNext->m_pose);
			pnodeNext = pnodeNext->m_pnodeParent;
		}
		boost::reverse(vecposef);
	} 
	return vecposef;
}