#pragma once

#include "dca.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

/**
  * @file viewer.h
  * @author julian 
  * @date 1/31/22
 */

namespace dca
{

class Viewer
{
public:
    explicit Viewer(const std::string &name);
    
    ~Viewer() = default;
    
    void add_pointcloud(std::string id, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, double size);
    
    void set_point_size(std::string id, double size);
    
    void
    add_normals(std::string id, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                const pcl::PointCloud<pcl::Normal>::Ptr &normals, int level, float scale);
    
    void show_viewer();
    
    void add_correspondences(const std::vector<std::vector<int>> &flann_indices,
                             const std::vector<std::vector<float>> &flann_distances,
                             const pcl::PointCloud<dca::DCADescriptor>::Ptr &dca_features,
                             const pcl::PointCloud<dca::DCADescriptor>::Ptr &trans_dca_features,
                             const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                             const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &trans_cloud, float dist_threshold);
    
    void add_point(const std::string &id, Eigen::Vector4d &matrix, double size, int r, int g, int b);

private:
    pcl::visualization::PCLVisualizer viewer;
};

}