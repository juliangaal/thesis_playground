
/**
  * @file viewer.cpp
  * @author julian 
  * @date 1/31/22
 */

#include "viewer.h"
#include <thread>

using namespace dca;

Viewer::Viewer(const std::string &name)
    : viewer(name)
{
    viewer.setBackgroundColor(0, 0, 0);
}

void Viewer::add_pointcloud(std::string id, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, double size)
{
    pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGBA>(cloud, rgb, id);
    set_point_size(id, size);
}

void Viewer::set_point_size(std::string id, double size)
{
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id);
}

void Viewer::add_normals(std::string id, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                         const pcl::PointCloud<pcl::Normal>::Ptr &normals, int level, float scale)
{
    viewer.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(cloud, normals, level, scale, id);
}

void Viewer::show_viewer()
{
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Viewer::add_correspondences(const std::vector<std::vector<int>> &flann_indices,
                                 const std::vector<std::vector<float>> &flann_distances,
                                 const pcl::PointCloud<dca::DCADescriptor>::Ptr &dca_features,
                                 const pcl::PointCloud<dca::DCADescriptor>::Ptr &trans_dca_features,
                                 const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                                 const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &trans_cloud, float dist_threshold)
{
    for (size_t i = 0; i < flann_indices.size(); ++i)
    {
        auto nni = flann_indices[i][0];
        auto dist = flann_distances[i][0];
        if (dist < dist_threshold)
        {
            // get index of point in trans_cloud from query features trans_dca_features
            const auto& p1_idx = trans_dca_features->points[i].point_idx;
            auto p1 = trans_cloud->points[p1_idx];
            // get index of point in cloud from database features dca_features
            const auto& p2_idx = dca_features->points[nni].point_idx;
            auto p2 = cloud->points[p2_idx];
            
            auto id = std::to_string(i);
            viewer.addLine(p1, p2, id);
        }
    }
}

void Viewer::add_point(const std::string &id, Eigen::Vector4d &matrix, double size, int r, int g, int b)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointXYZRGBA ptemp;
    ptemp.x = matrix.x();
    ptemp.y = matrix.y();
    ptemp.z = matrix.z();
    ptemp.r = r;
    ptemp.g = g;
    ptemp.b = b;
    ptemp.a = 255;
    temp->points.push_back(ptemp);
    add_pointcloud(id, temp, size);
}
