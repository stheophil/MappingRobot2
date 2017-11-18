#pragma once
#include "geometry.h"
#include <vector>

std::vector<rbt::point<double>> FindPath(cv::Mat matn, rbt::point<double> const& ptfStart, rbt::point<double> const& ptfEnd);
