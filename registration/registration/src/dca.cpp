#include "dca.h"
#include "util.h"

#include <pcl/console/time.h>
#include <pcl/registration/icp.h>
#include <random>
#include <eigen3/Eigen/Geometry>

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

int dca::generate_different_random_from_last(std::mt19937 &rng,
                                             std::uniform_int_distribution<std::mt19937::result_type> &dist, const std::vector<int> &last)
{
    int rand = dist(rng);
    bool dupe = (std::find(last.begin(), last.end(), rand) != last.end());
    while (dupe) {
        rand = dist(rng);
        dupe = (std::find(last.begin(), last.end(), rand) != last.end());
    }
    return rand;
}

Eigen::Matrix4f dca::filter_correspondences(const std::vector<std::vector<int>>& nn_indices, const std::vector<std::vector<float>>& nn_dists,
                                            const pcl::PointCloud<dca::DCADescriptor>::Ptr &dca_features,
                                            const pcl::PointCloud<dca::DCADescriptor>::Ptr &trans_dca_features,
                                            const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &trans_cloud,
                                            float dist_threshold, float flann_threshold)
{
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist(0,nn_indices.size()-1);
    
    double best_error = std::numeric_limits<double>::infinity();
    Eigen::Matrix4f best_transform = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZRGBA> inliers;
    pcl::PointCloud<pcl::PointXYZRGBA> outliers;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr feature_points(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr trans_feature_points(new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    for (auto i = 0u; i < nn_indices.size(); ++i)
    {
        const auto& trans_feature = trans_dca_features->points[i];
        trans_feature_points->push_back(trans_cloud->points[trans_feature.point_idx]);
        
        const auto& nni = nn_indices[i][0];
        const auto& feature = dca_features->points[nni];
        feature_points->push_back(cloud->points[feature.point_idx]);
    }
    
    pcl::console::TicToc time;
    time.tic();
    int iterations = 25;
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputSource(trans_feature_points);
    icp.setInputTarget(feature_points);
    icp.align(*trans_feature_points);
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;
    
    if (icp.hasConverged())
    {
        std::cout << "ICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "ICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        auto matrix = icp.getFinalTransformation ().cast<float>();
        printf ("Rotation matrix :\n");
        printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
        printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
        printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
        printf ("Translation vector :\n");
        printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
        
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::copyPointCloud(*trans_cloud, *transformed_cloud);
        pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, matrix);

        pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp2;
        icp2.setMaximumIterations(iterations/3);
        icp2.setInputSource(transformed_cloud);
        icp2.setInputTarget(cloud);
        icp2.align(*transformed_cloud);

        if (icp2.hasConverged ())
        {
            std::cout << "\nICP has converged, score is " << icp2.getFitnessScore () << std::endl;
            std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
            auto matrix2 = icp2.getFinalTransformation().cast<float>();
            printf ("Rotation matrix :\n");
            printf ("    | %6.3f %6.3f %6.3f | \n", matrix2 (0, 0), matrix2 (0, 1), matrix2 (0, 2));
            printf ("R = | %6.3f %6.3f %6.3f | \n", matrix2 (1, 0), matrix2 (1, 1), matrix2 (1, 2));
            printf ("    | %6.3f %6.3f %6.3f | \n", matrix2 (2, 0), matrix2 (2, 1), matrix2 (2, 2));
            printf ("Translation vector :\n");
            printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix2 (0, 3), matrix2 (1, 3), matrix2 (2, 3));
            return matrix2;
        }
        else
        {
            PCL_ERROR ("\nICP has not converged.\n");
        }
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
    }
    
    return best_transform;
//    int it = 0;
//    while(it != 100)
//    {
//        int corr1 = dist(rng);
//        int corr2 = generate_random_different_from_last(rng, dist, {corr1});
//        int corr3 = generate_random_different_from_last(rng, dist, {corr1, corr2});
//
//        if (nn_dists[corr1][0] > dist_threshold || nn_dists[corr2][0] > dist_threshold || nn_dists[corr3][0] > dist_threshold)
//        {
//            continue;
//        }
//
//        ++it;
//
//        // get corresponding points for umeyama
//        auto feature1 = trans_dca_features->points[corr1];
//        auto matching_feature1 = dca_features->points[nn_indices[corr1][0]];
//        auto point1 = trans_cloud->points[feature1.point_idx];
//        auto matching_point1 = cloud->points[matching_feature1.point_idx];
//
//        auto feature2 = trans_dca_features->points[corr2];
//        auto matching_feature2 = dca_features->points[nn_indices[corr2][0]];
//        auto point2 = trans_cloud->points[feature2.point_idx];
//        auto matching_point2 = cloud->points[matching_feature2.point_idx];
//
//        auto feature3 = trans_dca_features->points[corr3];
//        auto matching_feature3 = dca_features->points[nn_indices[corr3][0]];
//        auto point3 = trans_cloud->points[feature3.point_idx];
//        auto matching_point3 = cloud->points[matching_feature3.point_idx];
//
//        // setup umeyama
//        Eigen::MatrixXf src(4, 3); // homogeneous coords
//        Eigen::MatrixXf dst(4, 3); // homegeneous coords
//
//        src << point1.x, point2.x, point3.x,
//            point1.y, point2.y, point3.y,
//            point1.z, point2.z, point3.z,
//            1.0, 1.0, 1.0;
//
//        dst << matching_point1.x, matching_point2.x, matching_point3.x,
//            matching_point1.y, matching_point2.y, matching_point3.y,
//            matching_point1.z, matching_point2.z, matching_point3.z,
//            1.0, 1.0, 1.0;
//
//        Eigen::MatrixXf src_block = src.block<3,3>(0,0);
//        Eigen::MatrixXf dst_block = dst.block<3,3>(0,0);
//        Eigen::MatrixXf cR_t_umeyama = umeyama(src_block, dst_block);
//
//        Eigen::Matrix4f pcl_transform = Eigen::Matrix4f::Identity();
//        // rotation
//        pcl_transform(0,0) = cR_t_umeyama(0, 0);
//        pcl_transform(0,1) = cR_t_umeyama(0, 1);
//        pcl_transform(0,2) = cR_t_umeyama(0, 2);
//        pcl_transform(1,0) = cR_t_umeyama(1, 0);
//        pcl_transform(1,1) = cR_t_umeyama(1, 1);
//        pcl_transform(1,2) = cR_t_umeyama(1, 2);
//        pcl_transform(2,0) = cR_t_umeyama(2, 0);
//        pcl_transform(2,1) = cR_t_umeyama(2, 1);
//        pcl_transform(2,2) = cR_t_umeyama(2, 2);
//        pcl_transform(3, 0) = 0;
//        pcl_transform(3, 1) = 0;
//        pcl_transform(3, 2) = 0;
//        // translation
//        pcl_transform (0,3) = cR_t_umeyama(0, 3);
//        pcl_transform (1,3) = cR_t_umeyama(1, 3);
//        pcl_transform (2,3) = cR_t_umeyama(2, 3);
//        pcl_transform(3, 3) = 1;
//
//        pcl::PointCloud<pcl::PointXYZRGBA> transformed_cloud;
//        pcl::transformPointCloud(trans_feature_points, transformed_cloud, pcl_transform);
//
//        float error = 0.0f;
//        for (auto i = 0u; i < trans_feature_points.size(); ++i)
//        {
//            error += util::distance(transformed_cloud.points[i], feature_points.points[i]);
//        }
//
//        error /= static_cast<float>(trans_feature_points.size());
//
//        if (error < best_error)
//        {
//            best_error = error;
//            best_transform = pcl_transform;
//            // TODO apply transform for trans_feature_points directly?
//        }
//    }
//
//    std::cout << "best error: " << best_error << "\n";
//    std::cout << "best transform: \n" << best_transform << "\n";
}