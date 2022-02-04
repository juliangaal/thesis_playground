
/**
  * @file pcl2pcd.cpp
  * @author julian 
  * @date 2/4/22
 */

#include <chrono>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

ros::master::V_TopicInfo master_topics;
boost::filesystem::path folder;
std::string topic;
bool force;
constexpr const char* usage = "usage: rosrun utils pcl2pcd _output_folder:=foo _topic:=/bar (_force:=(false/true))";

void check_topic(const ros::master::V_TopicInfo& topics, const std::string& t)
{
    auto it = std::find_if(topics.begin(), topics.end(), [&](const auto& ti) { return ti.name == t; });
    if (it == topics.end())
    {
        ROS_WARN("Topic does not exist (yet?). Make sure that the data is published");
    }
}

void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    static int n_input = 0;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*input, cloud);
    
    if (cloud.empty())
    {
        ROS_WARN_STREAM("Received pointcloud is empty\n");
        return;
    }
    
    auto new_filename = folder / ("frame_" + std::to_string(n_input++) + ".pcd");
    if (boost::filesystem::exists(new_filename) && !force)
    {
        ROS_WARN_STREAM("File already exists");
        return;
    }
    
    pcl::io::savePCDFile(new_filename.string(), cloud);
    ROS_INFO_STREAM("Saved file " << new_filename.string());
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl2pcd_node");
    
    ros::NodeHandle nh("~");
    std::string param;
    
    if (!nh.getParam("output_folder", param))
    {
        ROS_ERROR("_output_folder must be set");
        ROS_ERROR(usage);
        return 1;
    }
    
    folder = boost::filesystem::path(param);
    if (!boost::filesystem::exists(folder) || !boost::filesystem::is_directory(folder))
    {
        ROS_ERROR_STREAM(param << " is not a valid path. It must exist and be a folder");
        return 1;
    }
    
    force = true;
    if (!nh.getParam("force", force))
    {
        ROS_WARN_STREAM("Choosing not to override pcd file if exists");
        force = false;
    }
    
    if (!nh.getParam("topic", topic))
    {
        ROS_ERROR("_topic must be set: listen to this topic.");
        ROS_ERROR(usage);
        return 1;
    }
    
    if (*(topic.begin()) != '/')
    {
        ROS_ERROR("Topic must begin with /");
        return 1;
    }
    
    ros::master::getTopics(master_topics);
    check_topic(master_topics, topic);
    
    ROS_INFO_STREAM("Options: \n" << std::boolalpha\
                                << "        force             : " << force << "\n"\
                                << "        saving to         : " << folder.string() << "\n"\
                                << "        reading from topic: " << topic << "\n");
    
    ros::Subscriber sub = nh.subscribe(topic, 1000, pcl_callback);
    ros::spin();
    
    return 0;
}