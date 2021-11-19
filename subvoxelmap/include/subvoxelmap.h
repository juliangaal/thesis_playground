#pragma once

#include "util.h"
#include <iostream>
#include <fmt/printf.h>
#include <cassert>

namespace map
{

class SubvoxelMap
{
public:
    SubvoxelMap(int w, int h, int d, double res, int map_size)
    : w(w), h(h), d(d), map(new double[h * w * d]), res(res), map_size(map_size)
    {
        assert(w > res);
        init_map();
    }

    SubvoxelMap(int map_size, double res)
    : w(static_cast<int>(map_size / res))
    , h(static_cast<int>(map_size / res))
    , d(static_cast<int>(map_size / res))
    , map(new double[h * w * d])
    , res(res)
    , map_size(map_size)
    {
        assert(w > res);
        init_map();
    }

    ~SubvoxelMap()
    {
        delete[] map;
    }

    SubvoxelMap& operator=(const SubvoxelMap&) = delete;
    SubvoxelMap& operator=(SubvoxelMap&&) = delete;
    SubvoxelMap(const SubvoxelMap&) = delete;
    SubvoxelMap(SubvoxelMap&&) = delete;

    double at_point(double x, double y, double z) const
    {
        util::Point<int> _3dindex = util::conv_3dpoint_3dindex(x, y, z, res);
        return at_index(_3dindex.x, _3dindex.y, _3dindex.z);
    }

    double at_index(int x, int y, int z) const
    {
        if (!in_range(x, y, z))
        {
            throw std::runtime_error(fmt::format("Accessing subvoxelmap @ invalid location: accessed @ ({}/{}/{}) with size {}\n", x, y, z, _size()));
        }
        int i = util::conv_3dindex_1dindex(x, y, z, w, d);
        return map[i];
    }

    // TODO auslagern
    void insert(double x, double y, double z, double val)
    {
        util::Point<int> _3dindex = util::conv_3dpoint_3dindex(x, y, z, res);
        if (!in_range(_3dindex.x, _3dindex.y, _3dindex.z))
        {
            throw std::runtime_error(fmt::format("Accessing subvoxelmap @ invalid location: accessed @ ({}/{}/{}) with size {}\n", x, y, z, _size()));
        }
        int i = util::conv_3dindex_1dindex(_3dindex.x, _3dindex.y, _3dindex.z, w, d);
        map[i] = val;
    }

    void clear()
    {
        init_map();
    }

    int _size() const
    {
        return h * w * d;
    }

    int _map_size() const
    {
        return map_size;
    }

    double _res() const
    {
        return res;
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

private:
    void init_map()
    {
        std::fill_n(map, h * w * d, NAN);
    }

    bool in_range(int x, int y, int z) const
    {
        return x >= 0 && x < w && y >= 0 && y < h && z >= 0 && z < d;
    }

    int w;
    int h;
    int d;
    double *map;
    double res;
    int map_size;
};

} // end namespace map
