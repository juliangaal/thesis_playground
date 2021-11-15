#pragma once

#include "util.h"
#include "subvoxelmap.h"
#include "parameters.h"
#include <fmt/printf.h>
#include <cassert>

namespace map
{

class Map : public subvoxelmap::Parameters
{
public:
    explicit Map(ros::NodeHandle& nh)
    : subvoxelmap::Parameters(nh)
    , h(static_cast<int>(map_size/map_res))
    , w(static_cast<int>(map_size/map_res))
    , d(static_cast<int>(map_size/map_res))
    , size(h * w * d)
    , res(map_res), map(new SubvoxelMap*[h * w * d])
    {
        ROS_DEBUG("%s", fmt::format("creating map with dims ({}/{}/{}), res: {}, total elements: {}\n", h, w, d, res, size).c_str());
    }

    ~Map()
    {
        for (int i = 0; i < h * w * d; ++i)
        {
            delete map[i];
        }

        delete[] map;
    }

    Map& operator=(const Map&) = delete;
    Map& operator=(Map&&) = delete;
    Map(const map::Map&) = delete;
    Map(Map&&) = delete;

    const SubvoxelMap* at(double x, double y, double z) const
    {
        return map[util::to_1d_arr_idx(x, y, z, res, w, d)];
    }

    const SubvoxelMap* at(Point p) const
    {
        return at(p.x, p.y, p.z);
    }

    const SubvoxelMap* at(int i) const
    {
        return at(i % w, ( i / w ) % h, i / ( w * h ));
    }

    void add(double x, double y, double z, int val)
    {
        if (submap_unititialized(x, y, z))
        {
            init_subvoxel_map(x, y, z);
            assert(map[util::to_1d_arr_idx(x, y, z, res, w, d)] != nullptr);
        }

        ROS_DEBUG("%s", fmt::format("creating submap @ ({}/{}/{}) for value: {} -> size: ({}/{}/{}), res: {}, total elements: {}\n", x / res, y / res, z / res, val, static_cast<int>(w / res), static_cast<int>(w / res), static_cast<int>(w / res), subvoxel_res, map[util::to_1d_arr_idx(x, y, z, res, w, d)]->_size()).c_str());
        insert_into_subvoxel_map(x, y, z, val);
    }

private:
    int h;
    int w;
    int d;
    int size;
    double res;
    SubvoxelMap** map;

    void init_subvoxel_map(double x, double y, double z)
    {
        int subvoxel_size = static_cast<int>(w / res);
        map[util::to_1d_arr_idx(x, y, z, res, w, d)] = new SubvoxelMap(subvoxel_size, subvoxel_size, subvoxel_size, subvoxel_res);
        assert(map[util::to_1d_arr_idx(x, y, z, res, w, d)] != nullptr);
    }

    void insert_into_subvoxel_map(double x, double y, double z, double val)
    {
        Point submap_p = to_submap_point(x, y, z);
        auto* subvoxelmap = map[util::to_1d_arr_idx(x, y, z, res, w, d)];
        subvoxelmap->insert(submap_p.x, submap_p.y, submap_p.z, val);
    }

    bool submap_unititialized(double x, double y, double z) const
    {
        return at(x, y, z) == nullptr;
    }

    Point to_submap_point(double x, double y, double z) const
    {
        return {x - (x / res), y - (y / res), z - (z / res)};
    }
};

} // end namespace map