#include "util.h"
#include "dca.h"

#include <thread>
#include <numeric>

void util::to_eigen(const std::vector<int>& indices, const flann::Matrix<double>& dataset, std::vector<pcl::PointXYZ> &neighbors, size_t number_neighbors)
{
    neighbors.resize(indices.size());

    auto bound = (number_neighbors <= indices.size() ? number_neighbors : indices.size());

    for (auto step = 0u; step < bound; ++step)
    {
        neighbors[step].x = dataset[indices[step]][0];
        neighbors[step].y = dataset[indices[step]][1];
        neighbors[step].z = dataset[indices[step]][2];
    }
}

void util::features2flannmatrix(const std::vector<std::vector<size_t>> &sorted_features_idx,
                                std::vector<pcl::PointCloud<dca::DCADescriptor>::Ptr> dca_features,
                                flann::Matrix<double> &points)
{
    if (sorted_features_idx.empty() or (sorted_features_idx.size() != dca_features.size()))
    {
        throw std::runtime_error("Invalid sized of features(_idx): features2flann");
    }

//    size_t size = std::accumulate(sorted_features_idx.begin(), sorted_features_idx.end(), 0, [&](const auto& s) { return s.size(); });
    size_t size = 0;
    for (const auto& s: sorted_features_idx)
    {
        size += s.size();
    }

    points = flann::Matrix<double>(new double[size * 3], size, 3);

    int i = 0;
    for (int j = 0; j < sorted_features_idx.size(); ++j)
    {
        const auto& indices = sorted_features_idx[j];
        const auto& features = dca_features[j];

        if (indices.size() != features->size())
        {
            throw std::runtime_error("Invalid sized of features(_idx): indices @ j");
        }

        for (const auto& k: indices)
        {
            auto& feature = features->points[k];
            points[i][0] = feature.curvature;
            points[i][1] = feature.neighbor_angle_sum;
            points[i][2] = feature.avg_neighbor_dist;
            std::cout << "added point to flann:" << points[i][0] << "/" << points[i][1] << "/" << points[i][2] << "\n";
        }

        ++i;
    }
}

float util::angle(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2)
{
    return safe_acos(p1.dot(p2) / (p1.norm() * p2.norm()));
}

float util::safe_acos(float f)
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

float util::distance(const pcl::PointXYZRGBA &p1, const pcl::PointXYZRGBA &p2)
{
    return (Eigen::Vector3f(p1.x, p1.y, p1.z) - Eigen::Vector3f(p2.x, p2.y, p2.z)).norm();
}
