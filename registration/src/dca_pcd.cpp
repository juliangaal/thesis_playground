#include "dca.h"
#include <iostream>

int main(void)
{
    pcl::visualization::CloudViewer viewer("cloud");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("../data/triangle_d1000.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }
    std::cout << "pcl version: " << PCL_VERSION_PRETTY << std::endl;
    std::cout << "Loaded point cloud with " << cloud->points.size() << " points\n";

    float threshold = 0.1;
    int k_neighbors = 5;

    pcl::PointCloud<DSADescriptor>::Ptr dsa_features(new pcl::PointCloud<DSADescriptor>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_dsa_features(new pcl::PointCloud<pcl::PointXYZRGBA>);

    calc_dsa_features(cloud, dsa_features, k_neighbors);
    std::cout << "Calculated " << dsa_features->size() << " features\n";

    filter_dsa_features(cloud, dsa_features, filtered_dsa_features, threshold);
    std::cout << "Kept " << threshold*100 << "% of features, " << filtered_dsa_features->size() << " in total\n";

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    float theta = M_PI/4; // The angle of rotation in radians
//    transform(0,0) = std::cos (theta);
//    transform(0,1) = -sin(theta);
//    transform(1,0) = sin (theta);
//    transform(1,1) = std::cos (theta);
    transform (0,3) = 2.5;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*cloud, *trans_cloud, transform);

    pcl::PointCloud<DSADescriptor>::Ptr trans_dsa_features(new pcl::PointCloud<DSADescriptor>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_filtered_dsa_features(new pcl::PointCloud<pcl::PointXYZRGBA>);

    calc_dsa_features(trans_cloud, trans_dsa_features, k_neighbors);
    std::cout << "Calculated " << trans_dsa_features->size() << " features\n";

    filter_dsa_features(trans_cloud, trans_dsa_features, trans_filtered_dsa_features, threshold);
    std::cout << "Kept " << threshold*100 << "% of features, " << trans_filtered_dsa_features->size() << " in total\n";

    show_cloud(viewer, {cloud, filtered_dsa_features, trans_cloud, trans_filtered_dsa_features});

    return 0;
}
