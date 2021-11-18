#pragma once

#include "util.h"
#include "subvoxelmap.h"
#include "parameters.h"
#include <fmt/printf.h>
#include <cassert>

namespace map
{

class Map
{
public:
    explicit Map(const subvoxelmap::Parameters& params)
    : Map(params.map_size, params.map_res, params.subvoxel_res)
    {}

    Map(int map_size, double map_res, double subvoxel_res)
    : map_size(map_size)
    , map_res(map_res)
    , h(static_cast<int>(map_size/map_res))
    , w(static_cast<int>(map_size/map_res))
    , d(static_cast<int>(map_size/map_res))
    , nelems(h * w * d)
    , map(new SubvoxelMap*[h * w * d])
    , subvoxel_res(subvoxel_res)
    {
        init_map();
    }

    ~Map()
    {
        for (int i = 0; i < h * w * d; ++i)
        {
            delete map[i];
        }

        delete[] map;
    }

    void clear()
    {
       throw std::runtime_error("NOT IMPLEMENTED");
    }

    Map& operator=(const Map&) = delete;
    Map& operator=(Map&&) = delete;
    Map(const map::Map&) = delete;
    Map(Map&&) = delete;

    const SubvoxelMap* at_index(int x, int y, int z) const
    {
        int i = util::conv_3dindex_1dindex(x, y, z, w, d);
        if (i > nelems)
        {
            throw std::runtime_error(fmt::format("Map accessed @ {} with size {}", i, nelems).c_str());
        }

        return map[i];
    }

    const SubvoxelMap* at_point(double x, double y, double z) const
    {
        int i = util::conv_3dpoint_1dindex(x, y, z, map_res, w, d);
        if (i > nelems || x < 0 || x >= map_size || y < 0 || y >= map_size || z < 0 || z >= map_size)
        {
            throw std::runtime_error(fmt::format("Map accessed with ({}/{}/{}) @ map[{}] with {} elements and map size {}", x, y, z, i, nelems, map_size).c_str());
        }

        return map[i];
    }

    double submap_at(double x, double y, double z) const
    {
        auto* submap = at_point(x, y, z);
        util::Point<double> in_submap = to_submap_point(x, y, z);
        util::Point<int> origin = util::conv_3dpoint_3dindex(x, y, z, map_res);
        return submap->at_point(in_submap.x, in_submap.y, in_submap.z);
    }

    const SubvoxelMap* at(int i) const
    {
        return at_index(i % w, (i / w) % h, i / (w * h));
    }

    bool insert(double x, double y, double z, int val)
    {
        if (!in_range(x, y, z))
        {
            return false;
        }

        if (submap_unititialized(x, y, z))
        {
            init_subvoxel_map(x, y, z);
            assert(map[util::conv_3dpoint_1dindex(x, y, z, map_res, w, d)] != nullptr);
        }

        util::Point<double> submap_p = to_submap_point(x, y, z);
        auto* subvoxelmap = map[util::conv_3dpoint_1dindex(x, y, z, map_res, w, d)];
        util::Point<int> _3dindex = util::conv_3dpoint_3dindex(x, y, z, map_res);
        insert_into_subvoxel_map(x, y, z, val);
        return true;
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

    double _map_res() const
    {
        return map_res;
    }

private:
    int map_size;
    double map_res;
    int h;
    int w;
    int d;
    int nelems;
    SubvoxelMap** map;
    double subvoxel_res;

    void init_map()
    {
        std::fill_n(map, h * w * d, nullptr);
    }

    bool in_range(double x, double y, double z) const
    {
        return x >= 0 && x < map_size && y >= 0 && y < map_size && z >= 0 && z < map_size;
    }

    void init_subvoxel_map(double x, double y, double z)
    {
        int subvoxel_elems_per_dim = static_cast<int>(map_res / subvoxel_res);
        map[util::conv_3dpoint_1dindex(x, y, z, map_res, w, d)] = new SubvoxelMap(subvoxel_elems_per_dim, subvoxel_elems_per_dim, subvoxel_elems_per_dim, subvoxel_res, map_res);
        assert(map[util::conv_3dpoint_1dindex(x, y, z, map_res, w, d)] != nullptr);
    }

    void insert_into_subvoxel_map(double x, double y, double z, double val)
    {
        auto* subvoxelmap = map[util::conv_3dpoint_1dindex(x, y, z, map_res, w, d)];

        util::Point<double> submap_p = to_submap_point(x, y, z);
        subvoxelmap->insert(submap_p.x, submap_p.y, submap_p.z, val);
    }

    bool submap_unititialized(double x, double y, double z) const
    {
        return at_point(x, y, z) == nullptr;
    }

    util::Point<double> to_submap_point(double x, double y, double z) const
    {
        util::Point<int> origin = util::conv_3dpoint_3dindex(x, y, z, map_res);
        return {x - origin.x, y - origin.y, z - origin.z};
    }
};

} // end namespace map