#pragma once

#include "dca.h"

#include <random>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>

namespace dca
{

struct DCADescriptor
{
    float curvature;
    float avg_neighbor_dist;
    float neighbor_angle_sum;
    size_t point_idx;
};

void calc_dca_features(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, Eigen::Vector4d &centroid, pcl::PointCloud<dca::DCADescriptor>::Ptr &features,
                       pcl::PointCloud<pcl::Normal>::Ptr &pcl_normals, int k_neighbors);

void sort_features_by_significance(pcl::PointCloud<dca::DCADescriptor>::Ptr& features);

void apply_color_2_features(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                            const pcl::PointCloud<dca::DCADescriptor>::Ptr &features, float threshold);

Eigen::Matrix4f filter_correspondences(const std::vector<std::vector<int>>& nn_indices, const std::vector<std::vector<float>>& nn_dists,
                                       const pcl::PointCloud<dca::DCADescriptor>::Ptr &dca_features,
                                       const pcl::PointCloud<dca::DCADescriptor>::Ptr &trans_dca_features,
                                       const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &trans_cloud,
                                       float dist_threshold, float flann_threshold);

int generate_different_random_from_last(std::mt19937& rng, std::uniform_int_distribution<std::mt19937::result_type>& dist, const std::vector<int> &last);

} // end namespace dca