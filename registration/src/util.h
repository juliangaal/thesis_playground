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
to_eigen(const std::vector<int> &indices, const flann::Matrix<double> &dataset, std::vector<pcl::PointXYZ> &neighbors,
         size_t number_neighbors);

float angle(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2);

float safe_acos(float f);

float distance(const pcl::PointXYZRGBA &p1, const pcl::PointXYZRGBA &p2);

void features2flannmatrix(const std::vector<std::vector<size_t>> &sorted_features_idx,
                          std::vector<pcl::PointCloud<dca::DSADescriptor>::Ptr> dca_features, flann::Matrix<double> &points);

}