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

template <typename T>
std::vector<T> merge_indices(std::vector<T> &v1, std::vector<T> &v2)
{
    v1.insert(v1.end(), std::make_move_iterator(v2.begin()),
              std::make_move_iterator(v2.end()));
    v2.erase(v2.begin(), v2.end());
    auto result = std::move(v1);
    return result;
}

template<typename T>
typename pcl::PointCloud<T>::Ptr merge_pointclouds(const typename pcl::PointCloud<T>::Ptr pcl1, typename pcl::PointCloud<T>::Ptr pcl2)
{
    typename pcl::PointCloud<T>::Ptr pcl3(new pcl::PointCloud<T>);
    *pcl3 = std::move(*pcl1);
    *pcl3 += std::move(*pcl2);
    return pcl3;
}

template<typename P, typename T>
void features2flannmatrix(const P &pcl, flann::Matrix<T> &dataset, float threshold)
{
    const auto& size = static_cast<size_t>(pcl->size() * threshold);
    dataset = flann::Matrix<T>(new T[size * 4], size, 4);

    for (auto i = 0u; i < size; ++i)
    {
        const auto& feature = pcl->points[i];
        dataset[i][0] = feature.curvature;
        dataset[i][1] = feature.avg_neighbor_dist;
        dataset[i][2] = feature.neighbor_angle_sum;
    }
}

}