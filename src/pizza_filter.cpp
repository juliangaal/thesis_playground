
/**
  * @file pizza_filter.cpp
  * @author julian 
  * @date 10/17/21
 */

#include "utils/ouster.h"
#include "utils/pizza_filter/parameters.h"

#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

const float INVALID_POINT_VALUE = std::numeric_limits<float>::quiet_NaN();

struct Overflow
{
    bool is_overflow;
    int amount;
};

struct PizzaFilter : public pizza_filter::Parameters
{
    explicit PizzaFilter(ros::NodeHandle& nh)
    : pizza_filter::Parameters(nh)
    , pcl_sub(nh.subscribe(sub_topic, 10, &PizzaFilter::cloud_handler, this))
    , pcl_pub(nh.advertise<sensor_msgs::PointCloud2>(pub_topic, 1000))
    , overflow{pizza_width + starting_angle > 360, std::abs(360 - static_cast<int>(pizza_width + starting_angle))}
    { 

    }
    
    ~PizzaFilter() override = default;
    
    void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {
        pcl::PointCloud<utils::PointOuster>::Ptr cloud(new pcl::PointCloud<utils::PointOuster>());
        pcl::fromROSMsg(*cloud_msg, *cloud);
    
        if (pcl_height * pcl_width != (int)cloud->size())
        {
            ROS_ERROR_STREAM_THROTTLE(1, "Check horizontal and vertical resolution. Height * Width must match pointclouds size (" << (int)cloud->size() << " vs " << pcl_height * pcl_width << ")");
            return;
        }
    
        float hresolution = (360.0f/static_cast<float>(pcl_width));
        int starting_w = static_cast<int>(starting_angle / hresolution);
        int ending_w = std::min(static_cast<int>(360.0f / hresolution), starting_w + static_cast<int>(pizza_width / hresolution));

        for (int h = 0; h < pcl_height; ++h)
        {
            if (overflow.is_overflow)
            {
                for (int w = 0; w < overflow.amount / hresolution && w < starting_w; ++w)
                {
                    auto &pt = cloud->points[h * pcl_width + w];
                    pt.x = INVALID_POINT_VALUE;
                    pt.y = INVALID_POINT_VALUE;
                    pt.z = INVALID_POINT_VALUE;
                    pt.intensity = 0;
                    pt.reflectivity = 0;
                }
            }
            for (int w = starting_w; w < ending_w; ++w)
            {
                auto &pt = cloud->points[h * pcl_width + w];
                pt.x = INVALID_POINT_VALUE;
                pt.y = INVALID_POINT_VALUE;
                pt.z = INVALID_POINT_VALUE;
                pt.intensity = 0;
                pt.reflectivity = 0;
            }
        }
        
        // convert back to ros cloud
        sensor_msgs::PointCloud2 pub_cloud;
        pcl::toROSMsg(*cloud, pub_cloud);
    
        // make sure header is correct and publish
        pub_cloud.header = cloud_msg->header;
        pcl_pub.publish(pub_cloud);
    }
    
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
    Overflow overflow;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pizza_filter");
    ros::NodeHandle nh;
    PizzaFilter worker(nh);
    ros::spin();
    return 0;
}