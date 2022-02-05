#include "dca.h"
#include "util.h"
#include "viewer.h"

#include <iostream>
#include <flann/flann.hpp>
#include <pcl/filters/voxel_grid.h>

void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


int main(int argc, char **argv)
{
    auto cloud_file = "/home/julian/desktop/128_foyer_1_stair/pcds/frame_5000.pcd";
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(cloud_file, *cloud) == -1)
    {
        std::cerr << "Couldn't read file " << cloud_file;
        return -1;
    }
    
    pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud);
    
    std::cout << "pcl version: " << PCL_VERSION_PRETTY << std::endl;
    std::cout << "Loaded point cloud with " << cloud->points.size() << " points\n";
    
    float threshold = 0.35;
    int k_neighbors = 100; // the higher density pointcloud, the fewer necessary
    
    Eigen::Vector4d centroid;
    pcl::PointCloud<dca::DCADescriptor>::Ptr dca_features(new pcl::PointCloud<dca::DCADescriptor>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    
    // Calculate dca features pointcloud 1
    dca::calc_dca_features(cloud, centroid, dca_features, normals, k_neighbors);
    std::cout << "Calculated " << dca_features->size() << " features\n";
    std::cout << "Calculated " << normals->size() << " normals\n";

    dca::sort_features_by_significance(dca_features);
    dca::apply_color_2_features(cloud, dca_features, threshold);
    std::cout << "Kept " << static_cast<int>(threshold*100) << "% of signicant features, " << static_cast<int>(dca_features->size()*threshold) << " in total\n";

    // load 2nd point cloud
    auto trans_cloud_file = "/home/julian/desktop/128_foyer_1_stair/pcds/frame_5001.pcd";
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(trans_cloud_file, *trans_cloud) == -1)
    {
        std::cerr << "Couldn't read file " << trans_cloud_file;
        return -1;
    }
    
    pcl::VoxelGrid<pcl::PointXYZRGBA> sor2;
    sor2.setInputCloud(trans_cloud);
    sor2.setLeafSize(0.05f, 0.05f, 0.05f);
    sor2.filter(*trans_cloud);
    
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    float theta = M_PI/8; // The angle of rotation in radians
//    transform(0,0) = std::cos (theta);
//    transform(0,1) = -sin(theta);
//    transform(1,0) = sin (theta);
//    transform(1,1) = std::cos (theta);
    transform (0,3) = 35;
//
    pcl::transformPointCloud(*trans_cloud, *trans_cloud, transform);
    
    // calculate dca features point cloud 2
    Eigen::Vector4d trans_centroid;
    pcl::PointCloud<dca::DCADescriptor>::Ptr trans_dca_features(new pcl::PointCloud<dca::DCADescriptor>);
    pcl::PointCloud<pcl::Normal>::Ptr trans_normals(new pcl::PointCloud<pcl::Normal>);

    dca::calc_dca_features(trans_cloud, trans_centroid, trans_dca_features, trans_normals, k_neighbors);
    std::cout << "Calculated " << trans_dca_features->size() << " features\n";
    std::cout << "Calculated " << trans_normals->size() << " normals\n";

    dca::sort_features_by_significance(trans_dca_features);
    dca::apply_color_2_features(trans_cloud, trans_dca_features, threshold);
    std::cout << "Kept " << static_cast<int>(threshold*100) << "% of signicant features, " << static_cast<int>(trans_dca_features->size()*threshold) << " in total\n";

    // flann
    flann::Matrix<float> dataset;
    auto flann_threshold = threshold / 3;
    std::cout << "using " << static_cast<int>(cloud->size() * flann_threshold) << " features (" << (flann_threshold * cloud->size())/cloud->size() << "%) for flann\n";
    util::features2flannmatrix(dca_features, dataset, flann_threshold);
    constexpr size_t point_limit = 10;
    flann::Index<flann::L2<float>> kdtree(dataset, flann::KDTreeSingleIndexParams(point_limit, false));
    kdtree.buildIndex();

    flann::Matrix<float> trans_dataset;
    util::features2flannmatrix(trans_dca_features, trans_dataset, flann_threshold);

    std::vector<std::vector<int>> indices;
    std::vector<std::vector<float>> dists;
    kdtree.knnSearch(trans_dataset, indices, dists, 1, flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED));
    std::cout << "Performed nearest neighbor search\n";
    
    // Create viewer and fill with relevant data
    float dist_threshold = 0.001;
    auto final_transform = dca::filter_correspondences(indices, dists, dca_features, trans_dca_features, cloud, trans_cloud, dist_threshold, flann_threshold);
    
    dca::Viewer viewer("PCL Viewer");
    viewer.add_pointcloud("cloud", cloud, 3.0);
    viewer.add_normals("normals", cloud, normals, 2, 0.02);
    viewer.add_pointcloud("trans cloud", trans_cloud, 3.0);
    viewer.add_normals("trans normals", trans_cloud, trans_normals, 1, 0.03);
    viewer.add_point("centroid", centroid, 6.0, 0, 255, 0);
    viewer.add_point("trans_centroid", trans_centroid, 6.0, 0, 255, 0);
    viewer.add_correspondences(indices, dists, dca_features, trans_dca_features, cloud, trans_cloud, dist_threshold);
    viewer.show_viewer();

    return 0;
}
