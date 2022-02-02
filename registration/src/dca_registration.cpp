#include "dca.h"
#include "util.h"
#include "viewer.h"

#include <iostream>
#include <flann/flann.hpp>

constexpr auto point_limit = 10u;

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

    // Calculate dca features pointcloud 1
    float threshold = 0.1;
    int k_neighbors = 10;
    
    std::vector<size_t> feature2points_idxs;
    pcl::PointCloud<dca::DSADescriptor>::Ptr dca_features(new pcl::PointCloud<dca::DSADescriptor>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_dca_features(new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    dca::calc_dca_features(cloud, feature2points_idxs, dca_features, normals, k_neighbors);
    std::cout << "Calculated " << dca_features->size() << " features\n";
    std::cout << "Calculated " << normals->size() << " normals\n";
    
    dca::sort_feature2point_idx_by_signifance(feature2points_idxs, dca_features);
    dca::apply_color_2_features(cloud, feature2points_idxs, filtered_dca_features, threshold);
    std::cout << "Kept " << threshold*100 << "% of features, " << filtered_dca_features->size() << " in total\n";

    // transform pointcloud 1
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    float theta = M_PI/2; // The angle of rotation in radians
    transform(0,0) = std::cos (theta);
    transform(0,1) = -sin(theta);
    transform(1,0) = sin (theta);
    transform(1,1) = std::cos (theta);
    transform (0,3) = 2.5;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::Normal>::Ptr trans_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::transformPointCloud(*cloud, *trans_cloud, transform);

    // calculate dca features point cloud 2
    std::vector<size_t> trans_feature2points_idxs;
    pcl::PointCloud<dca::DSADescriptor>::Ptr trans_dca_features(new pcl::PointCloud<dca::DSADescriptor>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_filtered_dca_features(new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    dca::calc_dca_features(trans_cloud, trans_feature2points_idxs, trans_dca_features, trans_normals, k_neighbors);
    std::cout << "Calculated " << trans_dca_features->size() << " features\n";
    std::cout << "Calculated " << trans_normals->size() << " normals\n";
    
    dca::sort_feature2point_idx_by_signifance(trans_feature2points_idxs, trans_dca_features);
    dca::apply_color_2_features(trans_cloud, trans_feature2points_idxs, trans_filtered_dca_features, threshold);
    std::cout << "Kept " << threshold*100 << "% of features, " << trans_filtered_dca_features->size() << " in total\n";

    // calculate correspondences
    flann::Matrix<double> dataset;
    util::features2flannmatrix({feature2points_idxs, trans_feature2points_idxs},
                               {dca_features, trans_dca_features},
                               dataset);

    // Construct an randomized kd-tree kdtree using a single kd-tree
    flann::Index<flann::L2<double>> kdtree(dataset, flann::KDTreeSingleIndexParams(point_limit, false));
    kdtree.buildIndex();

    std::vector<std::vector<int>> indices;
    std::vector<std::vector<double>> dists;
    kdtree.knnSearch(dataset, indices, dists, 1, flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED));

    for (auto i = 0; i < indices.size(); ++i)
    {
        auto nn_flann_idx = indices[i][0]; // only one neighbor -> 0
        auto nn_true_idx = nn_flann_idx % feature2points_idxs.size();

        std::cout << "closest point to: " << dataset[i][0] << "/" << dataset[i][1] << "/" << dataset[i][2] << " is: " << dataset[nn_flann_idx][0] << "/" << dataset[nn_flann_idx][1] << "/" << dataset[nn_flann_idx][2] << " with distance: " << dists[i][0] << "\n";
    }

    // Create viewer and fill with relevant data
    dca::Viewer viewer("PCL Viewer");
    viewer.add_pointcloud("sample cloud", cloud, 2.0);
    viewer.add_pointcloud("feature cloud", filtered_dca_features, 4.0);
    viewer.add_normals("normals", cloud, normals, 1, 0.03);
    viewer.add_pointcloud("trans sample cloud", trans_cloud, 2.0);
    viewer.add_pointcloud("trans feature cloud", trans_filtered_dca_features, 4.0);
    viewer.add_normals("trans normals", trans_cloud, trans_normals, 1, 0.03);
    viewer.show_viewer();

    return 0;
}
