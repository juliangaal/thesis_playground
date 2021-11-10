#pragma once

/**
  * @file parameters.h
  * @author julian 
  * @date 10/17/21
 */
 
#include <ros/node_handle.h>

namespace pizza_filter
{

struct Parameters
{
    explicit Parameters(ros::NodeHandle& nh) : sub_topic(), pub_topic(), pcl_height(), pcl_width(), starting_angle(), pizza_width()
    {
        nh.param<std::string>("pizza_filter/sub_topic", sub_topic, "/cloud");
        nh.param<std::string>("pizza_filter/pub_topic", pub_topic, "/cloud_out");
        ROS_INFO_STREAM("Listening for pcl @ " << sub_topic << " and publishing to " << pub_topic);
        nh.param<int>("pizza_filter/pcl_height", pcl_height, 128);
        nh.param<int>("pizza_filter/pcl_width", pcl_width, 1024);
        ROS_INFO_STREAM("Expecting pcl with " << pcl_width << "x" << pcl_height);
        nh.param<float>("pizza_filter/starting_angle", starting_angle, 128);
        nh.param<float>("pizza_filter/pizza_width", pizza_width, 30);
        
        if (starting_angle < 0 || starting_angle > 360)
        {
            throw std::runtime_error("starting_angle must be > 0 && < 360");
        }
    }
    
    virtual ~Parameters() = default;
    
    std::string sub_topic;
    std::string pub_topic;
    int pcl_height;
    int pcl_width;
    float starting_angle;
    float pizza_width;
};

} // end namespace pizza_filter