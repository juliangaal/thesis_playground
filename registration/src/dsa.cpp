#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>


void show_cloud(pcl::visualization::CloudViewer &viewer, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cloud)
{
    auto total = cloud[0];
    for (int i = 1; i < cloud.size(); ++i)
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

void
calculate_features(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr features, float threshold, RGBA color)
{
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
    ne.setSearchMethod(tree);
    
    std::cout << "built tree\n";
    
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.05);
    
    // Compute the features
    ne.compute(*normals);
    
    std::cout << "normals calculated\n";
    
    std::vector<PointN> points_and_normals;
    points_and_normals.resize(normals->size());
    
    for (size_t i = 0; i < normals->size(); ++i)
    {
//        cloud->points[i].r = 255;
//        cloud->points[i].g = 255;
//        cloud->points[i].b = 255;
//        cloud->points[i].a = 1;
        points_and_normals[i] = {cloud->points[i], normals->points[i]};
    }
    
    std::sort(points_and_normals.begin(), points_and_normals.end(), [&](const auto& pn1, const auto& pn2)
    {
        return pn1.normal.curvature > pn2.normal.curvature;
    });
    
    
    features->points.resize(points_and_normals.size() * threshold);
    for (size_t i = 0; i < features->points.size(); ++i)
    {
        features->points[i].x = points_and_normals[i].point.x;
        features->points[i].y = points_and_normals[i].point.y;
        features->points[i].z = points_and_normals[i].point.z;
        features->points[i].r = color.r;
        features->points[i].g = color.g;
        features->points[i].b = color.b;
    }
}


int main (void)
{
    pcl::visualization::CloudViewer viewer("cloud");
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("../data/triangle_d1000.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }
    std::cout << "Loaded point cloud with " << cloud->points.size() << " points\n";
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr features(new pcl::PointCloud<pcl::PointXYZRGBA>);
    float threshold = 0.1;
    calculate_features(cloud, features, threshold, {255, 0, 0, 255});
    
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    float theta = M_PI/4; // The angle of rotation in radians
    transform (0,0) = std::cos (theta);
    transform (0,1) = -sin(theta);
    transform (1,0) = sin (theta);
    transform (1,1) = std::cos (theta);
    transform (0,3) = 2.5;
    
    std::cout << "Transformation: \n" << transform << std::endl;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    
    show_cloud(viewer, {cloud, features});
    
    return 0;
}
