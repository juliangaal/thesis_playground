#pragma once

#include "dca.h"
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

void calc_dca_features(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<DCADescriptor>::Ptr features,
                       pcl::PointCloud<pcl::Normal>::Ptr pcl_normals, int k_neighbors);

void sort_features_by_significance(pcl::PointCloud<dca::DCADescriptor>::Ptr features);

void apply_color_2_features(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                            pcl::PointCloud<dca::DCADescriptor>::Ptr features, float threshold);

} // end namespace dca