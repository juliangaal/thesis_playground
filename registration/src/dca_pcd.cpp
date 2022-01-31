#include "dca.h"
#include "util.h"

#include <iostream>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("../data/triangle_d1000.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }
    std::cout << "pcl version: " << PCL_VERSION_PRETTY << std::endl;
    std::cout << "Loaded point cloud with " << cloud->points.size() << " points\n";
    
    float threshold = 0.1;
    int k_neighbors = 10;
    
    pcl::PointCloud<DSADescriptor>::Ptr dca_features(new pcl::PointCloud<DSADescriptor>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_dca_features(new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    calc_dca_features(cloud, dca_features, normals, k_neighbors);
    std::cout << "Calculated " << dca_features->size() << " features\n";
    std::cout << "Calculated " << normals->size() << " normals\n";

    filter_dca_features(cloud, dca_features, filtered_dca_features, threshold);
    std::cout << "Kept " << threshold*100 << "% of features, " << filtered_dca_features->size() << " in total\n";
    
//    auto viewer = util::normals_vis(cloud, normals);
//    util::show_cloud(viewer);
//    util::add_pcl(viewer, cloud);
//    util::add_normals(viewer, cloud, normals);
//    util::show_cloud(viewer);
//    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
////    float theta = M_PI/4; // The angle of rotation in radians
////    transform(0,0) = std::cos (theta);
////    transform(0,1) = -sin(theta);
////    transform(1,0) = sin (theta);
////    transform(1,1) = std::cos (theta);
//    transform (2,3) = 2.5;
////
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
//    pcl::transformPointCloud(*cloud, *trans_cloud, transform);
//
//    pcl::PointCloud<DSADescriptor>::Ptr trans_dca_features(new pcl::PointCloud<DSADescriptor>);
//    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_filtered_dca_features(new pcl::PointCloud<pcl::PointXYZRGBA>);
//
//    calc_dca_features(trans_cloud, trans_dca_features, k_neighbors);
//    std::cout << "Calculated " << trans_dca_features->size() << " features\n";
//
//    filter_dca_features(trans_cloud, trans_dca_features, trans_filtered_dca_features, threshold);
//    std::cout << "Kept " << threshold*100 << "% of features, " << trans_filtered_dca_features->size() << " in total\n";
    
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
    pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgb2(filtered_dca_features);
    viewer.addPointCloud<pcl::PointXYZRGBA> (filtered_dca_features, rgb2, "feature cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "feature cloud");
    viewer.addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal>(cloud, normals, 1);
    
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    
    return 0;
}
