#include "map.h"
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

void publish_subvoxelmap(const map::SubvoxelMap* map, ros::Publisher& pub, geometry_msgs::Point origin)
{
    static int i = 0;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "subvoxelmap";
    marker.id = ++i;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = map->_res();
    marker.scale.y = map->_res();
    marker.scale.z = map->_res();

    geometry_msgs::Point cube_center;

    for (int x = 0; x < map->_w(); ++x)
    {
        for (int y = 0; y < map->_h(); ++y)
        {
            for (int z = 0; z < map->_d(); ++z)
            {
                cube_center.x = origin.x + x * map->_res() + map->_res() / 2;
                cube_center.y = origin.y + y * map->_res() + map->_res() / 2;
                cube_center.z = origin.z + z * map->_res() + map->_res() / 2;
                marker.points.push_back(cube_center);

                std_msgs::ColorRGBA color;
                color.r = 1.0;
                color.g = 0.0;
                color.b = 0.0;
                color.a = 0.05;

                if (not std::isnan(map->at_index(x, y, z)))
                {
                    color.a = 1.0;
                }
                marker.colors.push_back(color);
            }
        }
    }

    pub.publish(marker);
}

void publish_map(const map::Map& map, ros::Publisher& pub)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "subvoxelmap";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = map._map_res();
    marker.scale.y = map._map_res();
    marker.scale.z = map._map_res();
    marker.color.a = 0.2;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    geometry_msgs::Point cube_center;

    for (int x = 0; x < map._w(); ++x)
    {
        for (int y = 0; y < map._h(); ++y)
        {
            for (int z = 0; z < map._d(); ++z)
            {
                auto* subvoxelmap = map.at_index(x, y, z);

                if (subvoxelmap == nullptr)
                {
                    cube_center.x = x + map._map_res() / 2;
                    cube_center.y = y + map._map_res() / 2;
                    cube_center.z = z + map._map_res() / 2;
                    marker.points.push_back(cube_center);

                }
                else
                {
                    cube_center.x = x;
                    cube_center.y = y;
                    cube_center.z = z;
                    publish_subvoxelmap(subvoxelmap, pub, cube_center);
                }
            }
        }
    }

    pub.publish(marker);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_test");
    ros::NodeHandle nh;
    subvoxelmap::Parameters params(nh);
    map::Map map(params);
    map.insert(0, 0, 2, 3);
    map.insert(0, 0, 2.6, 3);
    map.insert(2, 0, 0, 3);
    map.insert(2.6, 0, 0, 3);
    map.insert(2, 2, 2, 3);
    map.insert(2, 2.6, 2, 3);
    map.insert(2.6, 2.0, 2, 3);
    map.insert(2.6, 2.6, 2, 3);

    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("map", 0);

    while (ros::ok())
    {
        publish_map(map, vis_pub);
        ros::spinOnce();
        ros::Rate(0.1).sleep();
    }
}

