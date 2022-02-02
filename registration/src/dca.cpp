#include "dca.h"

float angle(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2)
{
    return safe_acos(p1.dot(p2) / (p1.norm() * p2.norm()));
}

float safe_acos(float f)
{
    if (f < -1.0f)
    {
        return M_PI;
    }
    
    if (f > 1.0f)
    {
        return 0.0f;
    }
    
    return std::acos(f);
}

float euclidean_distance(const pcl::PointXYZRGBA &p1, const pcl::PointXYZRGBA &p2)
{
    return (Eigen::Vector3f(p1.x, p1.y, p1.z) - Eigen::Vector3f(p2.x, p2.y, p2.z)).norm();
}

void calc_dca_features(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<size_t> &feature2point_idxs,
                       pcl::PointCloud<DSADescriptor>::Ptr features, pcl::PointCloud<pcl::Normal>::Ptr pcl_normals,
                       int k_neighbors)
{
    if (k_neighbors < 1)
    {
        throw std::runtime_error("invalid number of k neighbors of knn search");
    }
    
    Eigen::Vector4d centroid;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr demeaned_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::compute3DCentroid(*cloud, centroid);
    pcl::demeanPointCloud(*cloud, centroid, *demeaned_cloud);

    pcl::KdTreeFLANN<pcl::PointXYZRGBA> tree(new pcl::KdTreeFLANN<pcl::PointXYZRGBA>);
    tree.setInputCloud(demeaned_cloud);

    // variables for DSADescriptor calculation
    pcl::PointCloud<std::vector<int>> neighborhood;
    feature2point_idxs.resize(cloud->size());
    
    // variables for neighboorhood search
    pcl::PointCloud<pcl::PointXYZ> neighborhood_pcl;
    std::vector<int> point_idx_search(k_neighbors);
    std::vector<float> point_squared_distance(k_neighbors);
    
    for (auto &search_p: *demeaned_cloud)
    {
        if (tree.nearestKSearch(search_p, k_neighbors, point_idx_search, point_squared_distance) > 0)
        {
            neighborhood_pcl.points.resize(point_idx_search.size());
            for (size_t i = 0; i < neighborhood_pcl.points.size(); ++i)
            {
                neighborhood_pcl.points[i].x = demeaned_cloud->points[point_idx_search[i]].x;
                neighborhood_pcl.points[i].y = demeaned_cloud->points[point_idx_search[i]].y;
                neighborhood_pcl.points[i].z = demeaned_cloud->points[point_idx_search[i]].z;
            }

            Eigen::Vector4f normal;
            float curvature = 0;
            pcl::computePointNormal(neighborhood_pcl, normal, curvature);

            neighborhood.points.push_back(point_idx_search);
            pcl::Normal n(normal[0], normal[1], normal[2]);
            n.curvature = curvature;
            pcl_normals->points.push_back(n);
        }
    }

    features->points.resize(pcl_normals->points.size());

    for (size_t i = 0; i < demeaned_cloud->size(); ++i)
    {
        // Just for visuals
        cloud->points[i].r = 0;
        cloud->points[i].g = 255;
        cloud->points[i].b = 0;
        cloud->points[i].a = 255;
    
        const auto &point = demeaned_cloud->points[i];
        const auto &normal = pcl_normals->points[i];
        const auto &neighbors = neighborhood[i];
        if (neighbors.empty())
        {
            continue;
        }

        float neighbor_angle_sum = 0;
        float neighbor_dist_sum = 0;
    
        for (const auto &neighbor_idx: neighborhood[i])
        {
            const auto &neighbor = demeaned_cloud->points[neighbor_idx];
            neighbor_angle_sum += angle(Eigen::Vector3f(normal.normal), Eigen::Vector3f(pcl_normals->points[neighbor_idx].normal));
            neighbor_dist_sum += euclidean_distance(point, neighbor);
        }

        auto n_neighbors_f = static_cast<float>(neighborhood[i].size());
        float avg_neighbor_dist = neighbor_dist_sum / n_neighbors_f;
        features->points[i] = DSADescriptor{normal.curvature, avg_neighbor_dist, neighbor_angle_sum};
        feature2point_idxs[i] = i; // std::iota would be unnecessary loop, TODO do sorting of cloud directly
    }
}

void apply_color_2_features(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::vector<size_t> &feature2point_idxs,
                            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr significant_points, float threshold)
{
    significant_points->points.resize(feature2point_idxs.size() * threshold);
    for (size_t i = 0; i < significant_points->points.size(); ++i)
    {
        significant_points->points[i].x = cloud->points[feature2point_idxs[i]].x;
        significant_points->points[i].y = cloud->points[feature2point_idxs[i]].y;
        significant_points->points[i].z = cloud->points[feature2point_idxs[i]].z;
        significant_points->points[i].r = 255;
        significant_points->points[i].g = 0;
        significant_points->points[i].b = 0;
        significant_points->points[i].a = 255;
    }
}

void sort_feature2point_idx_by_signifance(std::vector <size_t> &feature2point_idxs,
                                          pcl::PointCloud<DSADescriptor>::Ptr features)
{
    std::sort(feature2point_idxs.begin(), feature2point_idxs.end(),
              [&](const size_t idx1, const size_t idx2)
              {
                  return features->points[idx1].curvature > features->points[idx2].curvature;
              });
}
