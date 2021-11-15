#include "map.h"
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "subvoxelmap");
    ros::NodeHandle nh;
    map::Map map(nh);
    map.add(0, 0, 2, 3);
    ros::spin();
}

