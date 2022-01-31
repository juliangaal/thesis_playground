
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

void Viewer::add_pointcloud(std::string id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, double size)
{
    pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGBA>(cloud, rgb, id);
    set_point_size(id, size);
}

void Viewer::set_point_size(std::string id, double size)
{
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id);
}

void Viewer::add_normals(std::string id, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                         pcl::PointCloud<pcl::Normal>::Ptr normals, int level, float scale)
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
