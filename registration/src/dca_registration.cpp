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
    
    pcl::PointCloud<dca::DCADescriptor>::Ptr dca_features(new pcl::PointCloud<dca::DCADescriptor>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    dca::calc_dca_features(cloud, dca_features, normals, k_neighbors);
    std::cout << "Calculated " << dca_features->size() << " features\n";
    std::cout << "Calculated " << normals->size() << " normals\n";

    dca::sort_features_by_significance(dca_features);
    dca::apply_color_2_features(cloud, dca_features, threshold);
    std::cout << "Kept " << threshold*100 << "% of features, " << dca_features->size()*threshold << " in total\n";

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
    pcl::PointCloud<dca::DCADescriptor>::Ptr trans_dca_features(new pcl::PointCloud<dca::DCADescriptor>);

    dca::calc_dca_features(trans_cloud, trans_dca_features, trans_normals, k_neighbors);
    std::cout << "Calculated " << trans_dca_features->size() << " features\n";
    std::cout << "Calculated " << trans_normals->size() << " normals\n";

    dca::sort_features_by_significance(trans_dca_features);
    dca::apply_color_2_features(trans_cloud, trans_dca_features, threshold);
    std::cout << "Kept " << threshold*100 << "% of features, " << dca_features->size()*threshold << " in total\n";

    // flann
    flann::Matrix<float> dataset;
    auto flann_threshold = threshold / 2;
    util::features2flannmatrix(dca_features, dataset, flann_threshold);
    size_t point_limit = 10;
    flann::Index<flann::L2<float>> kdtree(dataset, flann::KDTreeSingleIndexParams(point_limit, false));
    kdtree.buildIndex();

    flann::Matrix<float> trans_dataset;
    util::features2flannmatrix(trans_dca_features, trans_dataset, flann_threshold);

    std::vector<std::vector<int>> indices;
    std::vector<std::vector<float>> dists;
    kdtree.knnSearch(trans_dataset, indices, dists, 1, flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED));

    for (auto i = 0; i < indices.size(); ++i)
    {
        auto nni = indices[i][0];
        auto dist = dists[i][0];
        if (dist < 0.0001)
        {
            std::cout << "feature: " << trans_dca_features->points[i].curvature << " / "
                      << trans_dca_features->points[i].avg_neighbor_dist << " / "
                      << trans_dca_features->points[i].neighbor_angle_sum << \
            "with closest feature: " << dca_features->points[nni].curvature << " / "
                      << dca_features->points[nni].avg_neighbor_dist << " / "
                      << dca_features->points[nni].neighbor_angle_sum << \
            "with dist: " << dists[i][0] << "\n";

        }
    }

    // Create viewer and fill with relevant data
    dca::Viewer viewer("PCL Viewer");
    viewer.add_pointcloud("cloud", cloud, 3.0);
    viewer.add_normals("normals", cloud, normals, 2, 0.02);
    viewer.add_pointcloud("trans cloud", trans_cloud, 3.0);
    viewer.add_normals("trans normals", trans_cloud, trans_normals, 1, 0.03);
    viewer.show_viewer();

    return 0;
}
