#pragma once
#include "geometry.h"
#include <vector>

std::vector<rbt::point<double>> FindPath(cv::Mat matn, rbt::pose<double> const& posefStart, rbt::point<double> const& ptfEnd);
