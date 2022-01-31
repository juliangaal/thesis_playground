#pragma once

#include <iostream>
#include <flann/flann.hpp>
#include <eigen3/Eigen/Core>
#include "dca.h"

namespace util
{

using Point = Eigen::Matrix<double, 3, 1>;

void print_points_by_indices(const std::vector<std::vector<int>> &indices, const flann::Matrix<double> &dataset);

void
to_eigen(const std::vector<int> &indices, const flann::Matrix<double> &dataset, std::vector<Point> &neighbors,
         size_t number_neighbors);
}