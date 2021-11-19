#include <ros/ros.h>
#include <highfive/H5File.hpp>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>
#include <vector>
#include <fmt/printf.h>

#include "tsdf.h"

std::string map_file_name = "/home/julian/Documents/sim_map.h5";
constexpr int CHUNK_SHIFT = 6;
constexpr int CHUNK_SIZE = 1 << CHUNK_SHIFT;

constexpr int MAP_SHIFT = 6;
constexpr int MAP_RESOLUTION = 1 << MAP_SHIFT;

constexpr double truncation = 600;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tsdf_vis");
    ros::NodeHandle nh;

    HighFive::File f(map_file_name, HighFive::File::ReadOnly);
    HighFive::Group g = f.getGroup("/map");

    fmt::print("Read map...\n");

    size_t read_entries = 0;

    std::vector<geometry_msgs::Point> points;
    points.reserve(4'000'000);
    std::vector<std_msgs::ColorRGBA> colors;
    colors.reserve(4'000'000);

    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;
    color.a = 1;
    color.b = 0;

    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("tsdf_map", 1, true);

    // Fill the grid with the valid TSDF values of the map
    for (auto tag : g.listObjectNames())
    {
        // Get the chunk data
        HighFive::DataSet d = g.getDataSet(tag);
        std::vector<TSDFValue::RawType> chunk_data;
        d.read(chunk_data);
        // Get the chunk position
        std::vector<int> chunk_pos;
        std::string delimiter = "_";
        size_t pos = 0;
        std::string token;
        while ((pos = tag.find(delimiter)) != std::string::npos)
        {
            token = tag.substr(0, pos);
            chunk_pos.push_back(std::stoi(token));
            tag.erase(0, pos + delimiter.length());
        }
        chunk_pos.push_back(std::stoi(tag));

        for (int i = 0; i < CHUNK_SIZE && read_entries < 4'000'000; i++)
        {
            for (int j = 0; j < CHUNK_SIZE && read_entries < 4'000'000; j++)
            {
                for (int k = 0; k < CHUNK_SIZE && read_entries < 4'000'000; k++)
                {
                    auto entry = TSDFValue(chunk_data[CHUNK_SIZE * CHUNK_SIZE * i + CHUNK_SIZE * j + k]);

                    auto tsdf_value = (float)(entry.value());
                    auto weight = entry.weight();

                    int x = CHUNK_SIZE * chunk_pos[0] + i;
                    int y = CHUNK_SIZE * chunk_pos[1] + j;
                    int z = CHUNK_SIZE * chunk_pos[2] + k;

                    // Only touched cells are considered
                    if (weight > 0 && std::abs(tsdf_value) < truncation)
                    {
                        point.x = static_cast<double>(x) * MAP_RESOLUTION * 0.001;
                        point.y = static_cast<double>(y) * MAP_RESOLUTION * 0.001;
                        point.z = static_cast<double>(z) * MAP_RESOLUTION * 0.001;

                        if (tsdf_value >= 0)
                        {
                            color.r = tsdf_value / truncation;
                            color.g = 0;
                        }
                        else
                        {
                            color.r = 0;
                            color.g = -tsdf_value / truncation;
                        }


                        points.push_back(point);
                        colors.push_back(color);

                        // DO SOMETHING!
                        ++read_entries;
                    }
                }
            }
        }
    }

    fmt::print("Done\n");

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.ns = "map";
    marker.id = 0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.8 * MAP_RESOLUTION * 0.001;
    marker.points = std::move(points);
    marker.colors = std::move(colors);

    marker.header.stamp = ros::Time::now();

    pub.publish(marker);

    ros::Rate rate(0.1);

    fmt::print("Start publishing Markers\n");

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}