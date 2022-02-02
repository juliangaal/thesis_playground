#pragma once

#include "dca.h"
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>


struct PointN
{
    pcl::PointXYZRGBA point;
    pcl::Normal normal;
};

struct RGBA
{
    int r;
    int g;
    int b;
    int a;
};

struct DSAFeature
{
    pcl::PointXYZRGBA point;
    Eigen::Vector4f normal;
    float curvature;
    float avg_neighbor_dist;
};

struct Normal
{
    Eigen::Vector4f normal;
    float curvature;
};

struct NeighborHood
{
    float avg_neighbor_dist;
    std::vector<int> neighbor_idx;
};

struct DSADescriptor
{
    float curvature;
    float avg_neighbor_dist;
    float neighbor_angle_sum;
};

float angle(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2);

float safe_acos(float f);

float euclidean_distance(const pcl::PointXYZRGBA& p1, const pcl::PointXYZRGBA& p2);

void calc_dca_features(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<size_t> &feature2point_idxs,
                       pcl::PointCloud<DSADescriptor>::Ptr features, pcl::PointCloud<pcl::Normal>::Ptr pcl_normals,
                       int k_neighbors);

void sort_feature2point_idx_by_signifance(std::vector<size_t> &feature2point_idxs,
                                          pcl::PointCloud<DSADescriptor>::Ptr features);

void apply_color_2_features(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<size_t> &feature2point_idxs,
                            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr significant_points, float threshold);


