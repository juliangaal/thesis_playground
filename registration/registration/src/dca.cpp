#include "dca.h"
#include "util.h"

void
dca::calc_dca_features(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, Eigen::Vector4d &centroid, pcl::PointCloud<dca::DCADescriptor>::Ptr &features,
                       pcl::PointCloud<pcl::Normal>::Ptr &pcl_normals, int k_neighbors)
{
    if (k_neighbors < 1)
    {
        throw std::runtime_error("invalid number of k neighbors of knn search");
    }
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr demeaned_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::compute3DCentroid(*cloud, centroid);
    pcl::demeanPointCloud(*cloud, centroid, *demeaned_cloud);

    pcl::KdTreeFLANN<pcl::PointXYZRGBA> tree(new pcl::KdTreeFLANN<pcl::PointXYZRGBA>);
    tree.setInputCloud(demeaned_cloud);

    // variables for DCADescriptor calculation
    pcl::PointCloud<std::vector<int>> neighborhood;
    
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

            // TODO what happens when rotation around demeaned pointcloud? Shouldn't be necessary in theory, then
            if (normal.head<3>().dot(Eigen::Vector3f(centroid.x(), centroid.y(), centroid.z()).head<3>()) < 0)
            {
                normal = -normal;
            }

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
        cloud->points[i].r = 90;
        cloud->points[i].g = 90;
        cloud->points[i].b = 90;
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
            neighbor_angle_sum += util::angle(Eigen::Vector3f(normal.normal), Eigen::Vector3f(pcl_normals->points[neighbor_idx].normal));
            neighbor_dist_sum += util::distance(point, neighbor);
        }

        auto n_neighbors_f = static_cast<float>(neighborhood[i].size());
        float avg_neighbor_dist = neighbor_dist_sum / n_neighbors_f;
        features->points[i] = dca::DCADescriptor{normal.curvature, avg_neighbor_dist, neighbor_angle_sum, i};
    }
}

void dca::apply_color_2_features(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,
                                 const pcl::PointCloud<dca::DCADescriptor>::Ptr &features, float threshold)
{
    auto n_colored_points = static_cast<size_t>(features->points.size() * threshold);
    for (size_t i = 0; i < n_colored_points; ++i)
    {
        auto feature_idx = features->points[i].point_idx;
        cloud->points[feature_idx].r = 255;
        cloud->points[feature_idx].g = 0;
        cloud->points[feature_idx].b = 0;
        cloud->points[feature_idx].a = 255;
    }
}

void dca::sort_features_by_significance(pcl::PointCloud<dca::DCADescriptor>::Ptr& features)
{
    std::sort(features->begin(), features->end(),
              [&](const auto& feat1, const auto& feat2)
              {
                  return feat1.curvature > feat2.curvature;
              });
}
