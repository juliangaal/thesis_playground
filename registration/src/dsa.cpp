#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>

void show_cloud(pcl::visualization::CloudViewer &viewer, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cloud)
{
    auto total = cloud[0];
    for (size_t i = 1; i < cloud.size(); ++i)
    {
        *total += *cloud[i];
    }

    viewer.showCloud(total);
    while (!viewer.wasStopped()) {}
}

struct PointN
{
    pcl::PointXYZRGBA point;
    pcl::Normal normal;
};

struct RGBA
{
    int r;
    int g;
    int b;
    int a;
};

struct DSAFeature
{
    pcl::PointXYZRGBA point;
    Eigen::Vector4f normal;
    float curvature;
    float avg_neighbor_dist;
};

DSAFeature create_dsa_feature(const pcl::PointXYZRGBA& point, const pcl::PointCloud<pcl::PointXYZ>& neighborhood, float neighborhood_distance_sum)
{
    DSAFeature feature{point, {0, 0, 0, 0}, 0.0f, 0.0f};
    pcl::computePointNormal(neighborhood, feature.normal, feature.curvature);
    feature.avg_neighbor_dist = neighborhood_distance_sum / static_cast<float>(neighborhood.size());
    return feature;
}

void calc_dsa_features(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<DSAFeature>::Ptr features, float radius)
{
    pcl::KdTreeFLANN<pcl::PointXYZRGBA> tree(new pcl::KdTreeFLANN<pcl::PointXYZRGBA>);
    tree.setInputCloud(cloud);

    std::vector<int> point_idx_radius_search;
    std::vector<float> point_radius_squared_distance;

    pcl::PointCloud<pcl::PointXYZ> neighborhood;

    for (auto& search_p: *cloud) {
        // Just for visuals
        search_p.r = 50;
        search_p.g = 50;
        search_p.b = 50;
        search_p.a = 1;

        if (tree.radiusSearch(search_p, radius, point_idx_radius_search, point_radius_squared_distance) > 0) {
            neighborhood.points.resize(point_idx_radius_search.size());
            float neighborhood_distance_sum = 0;
            for (size_t i = 0; i < neighborhood.points.size(); ++i) {
                neighborhood.points[i].x = cloud->points[point_idx_radius_search[i]].x;
                neighborhood.points[i].y = cloud->points[point_idx_radius_search[i]].y;
                neighborhood.points[i].z = cloud->points[point_idx_radius_search[i]].z;
                neighborhood_distance_sum += point_radius_squared_distance[i];
            }
            features->points.emplace_back(create_dsa_feature(search_p, neighborhood, neighborhood_distance_sum));
        }
    }
}

void filter_dsa_features(pcl::PointCloud<DSAFeature>::Ptr features, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_features, float threshold)
{
    std::sort(features->points.begin(), features->points.end(), [&](const auto& pn1, const auto& pn2)
    {
        return pn1.curvature > pn2.curvature;
    });

    filtered_features->points.resize(features->points.size() * threshold);
    for (size_t i = 0; i < filtered_features->points.size(); ++i)
    {
        filtered_features->points[i].x = features->points[i].point.x;
        filtered_features->points[i].y = features->points[i].point.y;
        filtered_features->points[i].z = features->points[i].point.z;
        filtered_features->points[i].r = 255;
        filtered_features->points[i].g = 0;
        filtered_features->points[i].b = 0;
        filtered_features->points[i].a = 1;
    }
}

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
    float radius = 0.05;

    pcl::PointCloud<DSAFeature>::Ptr dsa_features(new pcl::PointCloud<DSAFeature>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_dsa_features(new pcl::PointCloud<pcl::PointXYZRGBA>);

    calc_dsa_features(cloud, dsa_features, radius);
    std::cout << "Calculated " << dsa_features->size() << " features\n";

    filter_dsa_features(dsa_features, filtered_dsa_features, threshold);
    std::cout << "Kept " << threshold*100 << "% of features, " << filtered_dsa_features->size() << " in total\n";

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    float theta = M_PI/4; // The angle of rotation in radians
    transform (0,0) = std::cos (theta);
    transform (0,1) = -sin(theta);
    transform (1,0) = sin (theta);
    transform (1,1) = std::cos (theta);
    transform (0,3) = 2.5;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*cloud, *trans_cloud, transform);

    pcl::PointCloud<DSAFeature>::Ptr trans_dsa_features(new pcl::PointCloud<DSAFeature>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_filtered_dsa_features(new pcl::PointCloud<pcl::PointXYZRGBA>);

    calc_dsa_features(trans_cloud, trans_dsa_features, radius);
    std::cout << "Calculated " << trans_dsa_features->size() << " features\n";

    filter_dsa_features(trans_dsa_features, trans_filtered_dsa_features, threshold);
    std::cout << "Kept " << threshold*100 << "% of features, " << trans_filtered_dsa_features->size() << " in total\n";

    show_cloud(viewer, {cloud, filtered_dsa_features, trans_cloud, trans_filtered_dsa_features});
    
    return 0;
}
