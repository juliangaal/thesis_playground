#include "dsa.h"

void show_cloud(pcl::visualization::CloudViewer &viewer, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> cloud)
{
    auto total = cloud[0];
    for (size_t i = 1; i < cloud.size(); ++i)
    {
        *total += *cloud[i];
    }

    viewer.showCloud(total);
    while (!viewer.wasStopped())
    {}
}

float angle(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2)
{
    return std::acos(p1.dot(p2) / (p1.norm() * p2.norm()));
}

float euclidean_distance(const pcl::PointXYZRGBA &p1, const pcl::PointXYZRGBA &p2)
{
    return std::sqrt(p1.x * p2.x + p1.y * p2.y + p1.z * p2.z);
}

void calc_dsa_features(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<DSADescriptor>::Ptr features,
                       int k_neighbors)
{
    if (k_neighbors < 1)
    {
        throw std::runtime_error("invalid number of k neighbors of knn search");
    }

    pcl::KdTreeFLANN<pcl::PointXYZRGBA> tree(new pcl::KdTreeFLANN<pcl::PointXYZRGBA>);
    tree.setInputCloud(cloud);

    // variables for DSADescriptor calculation
    pcl::PointCloud<std::vector<int>> neighborhood;
    pcl::PointCloud<Normal> normals;

    // variables for neighboorhood search
    pcl::PointCloud<pcl::PointXYZ> neighborhood_pcl;
    std::vector<int> point_idx_search(k_neighbors);
    std::vector<float> point_squared_distance(k_neighbors);

    for (auto &search_p: *cloud)
    {
        // Just for visuals
        search_p.r = 50;
        search_p.g = 50;
        search_p.b = 50;
        search_p.a = 1;

        if (tree.nearestKSearch(search_p, k_neighbors, point_idx_search, point_squared_distance) > 0)
        {
            neighborhood_pcl.points.resize(point_idx_search.size());
            for (size_t i = 0; i < neighborhood_pcl.points.size(); ++i)
            {
                neighborhood_pcl.points[i].x = cloud->points[point_idx_search[i]].x;
                neighborhood_pcl.points[i].y = cloud->points[point_idx_search[i]].y;
                neighborhood_pcl.points[i].z = cloud->points[point_idx_search[i]].z;
            }

            Normal normal;
            pcl::computePointNormal(neighborhood_pcl, normal.normal, normal.curvature);

            neighborhood.points.push_back(point_idx_search);
            normals.points.push_back(normal);
        }
    }

    features->points.resize(normals.points.size());

    for (int i = 0; i < cloud->size(); ++i)
    {
        const auto &point = cloud->points[i];
        const auto &normal = normals.points[i];
        const auto &neighbors = neighborhood[i];
        if (neighbors.empty())
        {
            continue;
        }

        float neighbor_angle_sum = 0;
        float neighbor_dist_sum = 0;

        for (const auto &neighbor_idx: neighborhood[i])
        {
            const auto &neighbor = cloud->points[i];
            neighbor_angle_sum += angle(normal.normal.head<3>(),
                                        normals.points[neighbor_idx].normal.head<3>());
            neighbor_dist_sum += euclidean_distance(point, neighbor);
        }

        float n_neighbors_f = static_cast<float>(neighborhood[i].size());
        float avg_neighbor_angle = neighbor_angle_sum / n_neighbors_f;
        float avg_neighbor_dist = neighbor_dist_sum / n_neighbors_f;
        features->points[i] = DSADescriptor{normal.curvature, avg_neighbor_dist, avg_neighbor_angle, i};
    }
}

void filter_dsa_features(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<DSADescriptor>::Ptr features,
                         pcl::PointCloud<pcl::PointXYZRGBA>::Ptr significant_points, float threshold)
{
    std::sort(features->points.begin(), features->points.end(), [&](const auto &pn1, const auto &pn2)
    {
        return pn1.curvature > pn2.curvature;
    });

    significant_points->points.resize(features->points.size() * threshold);
    for (size_t i = 0; i < significant_points->points.size(); ++i)
    {
        significant_points->points[i].x = cloud->points[features->points[i].point_idx].x;
        significant_points->points[i].y = cloud->points[features->points[i].point_idx].y;
        significant_points->points[i].z = cloud->points[features->points[i].point_idx].z;
        significant_points->points[i].r = 255;
        significant_points->points[i].g = 0;
        significant_points->points[i].b = 0;
        significant_points->points[i].a = 1;
    }
}