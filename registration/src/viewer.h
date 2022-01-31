#pragma once

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
    
    void add_pointcloud(std::string id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, double size);
    
    void set_point_size(std::string id, double size);
    
    void
    add_normals(std::string id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                pcl::PointCloud<pcl::Normal>::Ptr normals, int level, float scale);
    
    void show_viewer();

private:
    pcl::visualization::PCLVisualizer viewer;
};

}