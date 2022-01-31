#include "util.h"
#include "dca.h"
#include <thread>

void util::to_eigen(const std::vector<int>& indices, const flann::Matrix<double>& dataset, std::vector<util::Point>& neighbors, size_t number_neighbors)
{
    neighbors.resize(indices.size());

    auto bound = (number_neighbors <= indices.size() ? number_neighbors : indices.size());

    for (auto step = 0u; step < bound; ++step)
    {
        neighbors[step][0] = dataset[indices[step]][0];
        neighbors[step][1] = dataset[indices[step]][1];
        neighbors[step][2] = dataset[indices[step]][2];
    }

}