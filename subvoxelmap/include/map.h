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
    , nelems(h * w * d)
    , res(map_res), map(new SubvoxelMap*[h * w * d])
    {
        ROS_DEBUG("%s", fmt::format("creating map with dims ({}/{}/{}), res: {}, total elements: {}\n", h, w, d, res, nelems).c_str());
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

    const SubvoxelMap* at(int x, int y, int z) const
    {
        int i = util::conv_3dindex_1dindex(x, y, z, w, d);
        if (i > nelems)
        {
            throw std::runtime_error(fmt::format("Map accessed @ {} with size {}", i, nelems).c_str());
        }

        return map[i];
    }

    const SubvoxelMap* at(double x, double y, double z) const
    {
        int i = util::conv_3dpoint_1dindex(x, y, z, res, w, d);
        if (i > nelems || x < 0 || x >= map_size || y < 0 || y >= map_size || z < 0 || z >= map_size)
        {
            throw std::runtime_error(fmt::format("Map accessed with ({}/{}/{}) @ map[{}] with {} elements and map size {}", x, y, z, i, nelems, map_size).c_str());
        }

        return map[i];
    }

    const SubvoxelMap* at(int i) const
    {
        return at(i % w, ( i / w ) % h, i / ( w * h ));
    }

    void insert(double x, double y, double z, int val)
    {
        if (!in_range(x, y, z))
        {
            ROS_ERROR("%s", fmt::format("Received point out of bounds: ({}/{}/{})", x, y, z).c_str());
            return;
        }

        if (submap_unititialized(x, y, z))
        {
            init_subvoxel_map(x, y, z);
            assert(map[util::conv_3dpoint_1dindex(x, y, z, res, w, d)] != nullptr);
        }

        util::Point<double> submap_p = to_submap_point(x, y, z);
        auto* subvoxelmap = map[util::conv_3dpoint_1dindex(x, y, z, res, w, d)];
        util::Point<int> _3dindex = util::conv_3dpoint_3dindex(x, y, z, res);
        ROS_DEBUG("%s", "--\n");
        ROS_DEBUG("%s", fmt::format("Inserting    ({}/{}/{})", x, y, z).c_str());
        ROS_DEBUG("%s", fmt::format("Converted to ({}/{}/{})", _3dindex.x, _3dindex.y, _3dindex.z).c_str());
        ROS_DEBUG("%s", fmt::format("Creating submap with dims ({}/{}/{}) and res {}", subvoxelmap->_h(), subvoxelmap->_w(), subvoxelmap->_d(), subvoxel_res).c_str());
        ROS_DEBUG("%s", fmt::format("Converting ({}/{}/{}) to ({}/{}/{}) to insert into submap", x, y, z, submap_p.x, submap_p.y, submap_p.z).c_str());
        insert_into_subvoxel_map(x, y, z, val);
    }

    int _h() const
    {
        return h;
    }

    int _w() const
    {
        return w;
    }

    int _d() const
    {
        return d;
    }

    double _res() const
    {
        return res;
    }

private:
    int h;
    int w;
    int d;
    int nelems;
    double res;
    SubvoxelMap** map;

    bool in_range(double x, double y, double z)
    {
        return x >= 0 && x < map_size && y >= 0 && y < map_size && z >= 0 && z < map_size;
    }

    void init_subvoxel_map(double x, double y, double z)
    {
        int subvoxel_elems_per_dim = static_cast<int>(res / subvoxel_res);
        map[util::conv_3dpoint_1dindex(x, y, z, res, w, d)] = new SubvoxelMap(subvoxel_elems_per_dim, subvoxel_elems_per_dim, subvoxel_elems_per_dim, subvoxel_res, res);
        assert(map[util::conv_3dpoint_1dindex(x, y, z, res, w, d)] != nullptr);
    }

    void insert_into_subvoxel_map(double x, double y, double z, double val)
    {
        auto* subvoxelmap = map[util::conv_3dpoint_1dindex(x, y, z, res, w, d)];

        util::Point<double> submap_p = to_submap_point(x, y, z);
        subvoxelmap->insert(submap_p.x, submap_p.y, submap_p.z, val);
    }

    bool submap_unititialized(double x, double y, double z) const
    {
        return at(x, y, z) == nullptr;
    }

    util::Point<double> to_submap_point(double x, double y, double z) const
    {
        util::Point<int> origin = util::conv_3dpoint_3dindex(x, y, z, res);
        return {x - origin.x, y - origin.y, z - origin.z};
    }
};

} // end namespace map