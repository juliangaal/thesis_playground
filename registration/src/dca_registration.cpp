#include "dca.h"
#include "viewer.h"

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
    
    std::vector<size_t> feature2points_idxs;
    pcl::PointCloud<DSADescriptor>::Ptr dca_features(new pcl::PointCloud<DSADescriptor>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_dca_features(new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    calc_dca_features(cloud, feature2points_idxs, dca_features, normals, k_neighbors);
    std::cout << "Calculated " << dca_features->size() << " features\n";
    std::cout << "Calculated " << normals->size() << " normals\n";
    
    sort_feature2point_idx_by_signifance(feature2points_idxs, dca_features);
    apply_color_2_features(cloud, feature2points_idxs, filtered_dca_features, threshold);
    std::cout << "Kept " << threshold*100 << "% of features, " << filtered_dca_features->size() << " in total\n";
    
    dca::Viewer viewer("PCL Viewer");
    viewer.add_pointcloud("sample cloud", cloud, 2.0);
    viewer.add_pointcloud("feature cloud", filtered_dca_features, 4.0);
    viewer.add_normals("normals", cloud, normals, 1, 0.03);
    
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    float theta = M_PI/8; // The angle of rotation in radians
    transform(0,0) = std::cos (theta);
    transform(0,1) = -sin(theta);
    transform(1,0) = sin (theta);
    transform(1,1) = std::cos (theta);
    transform (0,3) = 1.5;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr trans_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::transformPointCloud(*cloud, *trans_cloud, transform);
    
    std::vector<size_t> trans_feature2points_idxs;
    pcl::PointCloud<DSADescriptor>::Ptr trans_dca_features(new pcl::PointCloud<DSADescriptor>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_filtered_dca_features(new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    calc_dca_features(trans_cloud, trans_feature2points_idxs, trans_dca_features, trans_normals, k_neighbors);
    std::cout << "Calculated " << trans_dca_features->size() << " features\n";
    std::cout << "Calculated " << trans_normals->size() << " normals\n";
    
    sort_feature2point_idx_by_signifance(trans_feature2points_idxs, trans_dca_features);
    apply_color_2_features(trans_cloud, trans_feature2points_idxs, trans_filtered_dca_features, threshold);
    std::cout << "Kept " << threshold*100 << "% of features, " << trans_filtered_dca_features->size() << " in total\n";
    
    viewer.add_pointcloud("trans sample cloud", trans_cloud, 2.0);
    viewer.add_pointcloud("trans feature cloud", trans_filtered_dca_features, 4.0);
    viewer.add_normals("trans normals", trans_cloud, trans_normals, 1, 0.03);
    viewer.show_viewer();
    
    return 0;
}
