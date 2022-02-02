#include "dca.h"
#include "viewer.h"

#include <iostream>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr unprocessed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("../data/carpark_cloud_velodyne_hdl_32e.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }
    std::cout << "pcl version: " << PCL_VERSION_PRETTY << std::endl;
    std::cout << "Loaded point cloud with " << cloud->points.size() << " points\n";
    
    float threshold = 0.1;
    int k_neighbors = 25;
    
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
    viewer.add_pointcloud("sample cloud", cloud, 1.0);
    viewer.add_pointcloud("feature cloud", filtered_dca_features, 4.0);
    viewer.add_normals("normals", cloud, normals, 1, 0.03);
    viewer.show_viewer();
    
    return 0;
}
