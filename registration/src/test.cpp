#include "dca.h"
#include "util.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <numeric>

TEST_CASE("Tests", "[tests]")
{
    {
        Eigen::Vector3f p1{0, 2, 2};
        REQUIRE(p1.norm() * p1.norm() == Catch::Approx(8.0f).epsilon(0.001));

        Eigen::Vector3f p2{1, 2, 2};
        auto dot = p1.dot(p2);
        REQUIRE(dot == Catch::Approx(8.f).epsilon(0.001));
    }
    {
        Eigen::Vector3f p1{1, 1, 1};
        Eigen::Vector3f p2{-1, -1, -1};
        REQUIRE(util::angle(p1, p2) == Catch::Approx(M_PI).epsilon(0.001));
        REQUIRE(util::angle(p2, p1) == Catch::Approx(M_PI).epsilon(0.001));
    }
    {
        Eigen::Vector3f p1{1, 0, 0};
        Eigen::Vector3f p2{0, 1, 0};
        REQUIRE(util::angle(p1, p2) == Catch::Approx(M_PI/2.f).epsilon(0.001));
    }
}

TEST_CASE("FLANN", "[flann]")
{
    size_t n_points = 10;

    // dca specific test for NN search
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr features1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr features2(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<float> gen(n_points);
    std::iota(gen.begin(), gen.end(), 0);

    // indices for each point cloud. Will be sorted
    std::vector<size_t> pcl1_2_features1_indices(n_points);
    std::iota(pcl1_2_features1_indices.begin(), pcl1_2_features1_indices.end(), 0);
    std::vector<size_t> pcl2_2_features2_indices(n_points);
    std::iota(pcl2_2_features2_indices.begin(), pcl2_2_features2_indices.end(), 0);

    // fill points with generated data
    pcl1->points.resize(n_points);
    pcl2->points.resize(n_points);
    features1->points.resize(n_points);
    features2->points.resize(n_points);

    for (int i = 0; i < n_points; ++i)
    {
        pcl1->points[i].x = -gen[i];
        pcl1->points[i].y = -gen[i];
        pcl1->points[i].z = -gen[i];
        pcl2->points[i].x = -gen[i] + 0.1;
        pcl2->points[i].y = -gen[i] + 0.1;
        pcl2->points[i].z = -gen[i] + 0.1;
        features1->points[i].x = gen[i];
        features1->points[i].y = gen[i];
        features1->points[i].z = gen[i];
        features2->points[i].x = gen[i] + 0.1;
        features2->points[i].y = gen[i] + 0.1;
        features2->points[i].z = gen[i] + 0.1;
    }

    std::sort(pcl1_2_features1_indices.begin(), pcl1_2_features1_indices.end(), [&](const auto& a, const auto& b){ return features1->points[a].x > features1->points[b].x; });
    std::sort(pcl2_2_features2_indices.begin(), pcl2_2_features2_indices.end(), [&](const auto& a, const auto& b){ return features2->points[a].x > features2->points[b].x; });

    // verify if sorted
    for (auto i = 0; i < n_points; ++i)
    {
        auto ri = (n_points-1) - i;
        REQUIRE(features1->points[pcl1_2_features1_indices[i]].x == Catch::Approx(gen[ri]).epsilon(0.001));
        REQUIRE(features2->points[pcl2_2_features2_indices[i]].x == Catch::Approx(gen[ri] + 0.1).epsilon(0.001));
    }

    flann::Matrix<float> dataset;
    util::features2flannmatrix(features1, dataset, 0);
    size_t point_limit = 10;
    flann::Index<flann::L2<float>> kdtree(dataset, flann::KDTreeSingleIndexParams(point_limit, false));
    kdtree.buildIndex();

    flann::Matrix<float> query_dataset;
    util::features2flannmatrix(features2, query_dataset, 0);

    std::vector<std::vector<int>> indices;
    std::vector<std::vector<float>> dists;
    kdtree.knnSearch(query_dataset, indices, dists, 1, flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED));

    REQUIRE(indices.size() == features1->size());

    for (auto i = 0; i < indices.size(); ++i)
    {
        auto nn = indices[i][0];
        std::cout << "nn of " << i << ": " << nn << " @ " << pcl
//        std::cout << "query " << i << ":\n  feature" << features2->points[i] << " is closest to " << features1->points[nn] << "\n";
//        std::cout << "  and matches " << pcl2->points[pcl2_2_features2_indices[indices[i][0]]] << " to point " << pcl1_2_features1_indices[indices[i][0]] << "\n";
    }

}
