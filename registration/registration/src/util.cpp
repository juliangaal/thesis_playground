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
