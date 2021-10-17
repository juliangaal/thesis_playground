
/**
  * @file pcl_hline_reducer.cpp
  * @author julian 
  * @date 10/17/21
 */
 
#include "utils/ouster.h"
#include "utils/pcl_hline_reducer/parameters.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

struct PCLHlineReducer : public pcl_hline_reducer::Parameters
{
    explicit PCLHlineReducer(ros::NodeHandle& nh)
    : pcl_hline_reducer::Parameters(nh)
    , pcl_sub(nh.subscribe(sub_topic, 10, &PCLHlineReducer::cloud_handler, this))
    , pcl_pub(nh.advertise<sensor_msgs::PointCloud2>(pub_topic, 1000))
    {}
    
    ~PCLHlineReducer() override = default;
    
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
    
    void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {
        // convert cloud to pcl to be able to iterate through it more easily
        pcl::PointCloud<utils::PointOuster>::Ptr cloud(new pcl::PointCloud<utils::PointOuster>());
        pcl::fromROSMsg(*cloud_msg, *cloud);
        
        // init target cloud
        pcl::PointCloud<utils::PointOuster> result_cloud;
        result_cloud.points.resize(pcl_target_height * pcl_width);
        
        if (pcl_height * pcl_width != (int)cloud->size())
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Check horizontal and vertical resolution. Height * Width must match pointclouds size (" << (int)cloud->size() << " vs " << pcl_height * pcl_width << ")");
            return;
        }
        
        // target height index has to be handled seperately
        int target_u = 0;
        
        for (int u = 0; u < pcl_height; u++)
        {
            // If line isn't supposed to skipped
            // copy point of original cloud into correct spot in new cloud
            if (u % skip == 0)
            {
                for (int v = 0; v < pcl_width; v++)
                {
                    const auto &pt = cloud->points[u * pcl_width + v];
                    result_cloud.points[target_u * pcl_width + v] = pt;
                }
    
                target_u++;
            }
        }
        
        // convert subsampled cloud to sensor_msgs::PointCloud
        sensor_msgs::PointCloud2 pub_cloud;
        pcl::toROSMsg(result_cloud, pub_cloud);
        
        // make sure header is correct and publish
        pub_cloud.header = cloud_msg->header;
        pcl_pub.publish(pub_cloud);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_hline_reducer");
    ros::NodeHandle nh;
    PCLHlineReducer worker(nh);
    ros::spin();
    return 0;
}