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
    Parameters(ros::NodeHandle& nh) : sub_topic(), pub_topic(), pcl_height(), pcl_width(), pcl_target_height(), skip()
    {
        nh.param<std::string>("pcl_hline_reducer/sub_topic", sub_topic, "/cloud");
        nh.param<std::string>("pcl_hline_reducer/pub_topic", pub_topic, "/cloud_out");
        ROS_INFO_STREAM("Listening for pcl @ " << sub_topic << " and publishing to " << pub_topic);
        nh.param<int>("pcl_hline_reducer/pcl_height", pcl_height, 128);
        nh.param<int>("pcl_hline_reducer/pcl_width", pcl_width, 1024);
        ROS_INFO_STREAM("Expecting pcl with " << pcl_width << "x" << pcl_height);
        nh.param<int>("pcl_hline_reducer/pcl_target_height", pcl_target_height, 64);
        ROS_INFO_STREAM("Reducing target height to " << pcl_target_height);
        
        if (pcl_height / pcl_target_height % 2 != 0)
        {
            throw std::runtime_error("height must be divisible by target_height");
        }
        
        if (pcl_target_height >= pcl_height)
        {
            throw std::runtime_error("target_height must be smaller that height");
        }
        
        skip = pcl_height / pcl_target_height;
    }
    
    virtual ~Parameters() = default;
    
    std::string sub_topic;
    std::string pub_topic;
    int pcl_height;
    int pcl_width;
    int pcl_target_height;
    int skip;
};

} // end namespace pcl_hline_reducer
