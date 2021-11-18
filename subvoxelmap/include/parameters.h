#pragma once

/**
  * @file parameters.h
  * @author julian 
  * @date 10/17/21
 */
 
#include <ros/node_handle.h>

namespace subvoxelmap
{

struct Parameters
{
    Parameters(int map_size, double map_res, double subvoxel_res, bool debug = false)
    : map_size(map_size)
    , map_res(map_res)
    , subvoxel_res(subvoxel_res)
    , debug(debug)
    {
    }

    explicit Parameters(ros::NodeHandle& nh) : map_size(), map_res(), subvoxel_res()
    {
        nh.param<int>("subvoxelmap/map_size", map_size, 10);
        nh.param<double>("subvoxelmap/map_res", map_res, 2.0);
        ROS_INFO_STREAM("Map config: size " << map_size << ", map_res " << map_res);
        nh.param<double>("subvoxelmap/subvoxel_res", subvoxel_res, 1.0);
        ROS_INFO_STREAM("Subvoxel config: map_res " << subvoxel_res);
        nh.param<bool>("subvoxelmap/debug", debug, false);
        ROS_INFO_STREAM("Debug output: " << std::boolalpha << debug);

        if (debug)
        {
            ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
        }

        if (map_size < 1)
        {
            throw std::runtime_error("starting_angle must be > 0 && < 360");
        }

        if (map_res == 0 || map_res >= map_size)
        {
            throw std::runtime_error("map_res must be > 0 and < mapsize");
        }

        if (subvoxel_res == 0 || subvoxel_res >= map_res)
        {
            throw std::runtime_error("subvoxel_res must be > 0 and < mapsize");
        }
    }
    
    ~Parameters() = default;
    
    int map_size;
    double map_res;
    double subvoxel_res;
    bool debug;
};

} // end namespace pizza_filter