#pragma once

/**
  * @file parameters.h
  * @author julian 
  * @date 10/17/21
 */

#include <string>
#include <ros/node_handle.h>

namespace pcl_hline_reducer
{

struct Parameters
{
    Parameters(ros::NodeHandle& nh) : sub_topic(), pub_topic(), height(), width(), target_height(), skip()
    {
        nh.param<std::string>("pcl_hline_reducer/sub_topic", sub_topic, "/cloud");
        nh.param<std::string>("pcl_hline_reducer/pub_topic", pub_topic, "/cloud_out");
        ROS_INFO_STREAM("Listening for pcl @ " << sub_topic << " and publishing to " << pub_topic);
        nh.param<int>("pcl_hline_reducer/height", height, 128);
        nh.param<int>("pcl_hline_reducer/width", width, 1024);
        ROS_INFO_STREAM("Expecting pcl with " << width << "x" << height);
        nh.param<int>("pcl_hline_reducer/target_height", target_height, 64);
        ROS_INFO_STREAM("Reducing target height to " << target_height);
        
        if (height / target_height % 2 != 0)
        {
            throw std::runtime_error("height must be divisable my target_height");
        }
        
        if (target_height >= height)
        {
            throw std::runtime_error("target_height must be smaller that height");
        }
        
        skip = height / target_height;
    }
    
    virtual ~Parameters() = default;
    
    std::string sub_topic;
    std::string pub_topic;
    int height;
    int width;
    int target_height;
    int skip;
};

} // end namespace pcl_hline_reducer
