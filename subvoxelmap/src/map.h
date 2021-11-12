#pragma once

#include "util.h"
#include "subvoxelmap.h"
#include <fmt/printf.h>
#include <cassert>

namespace map
{

class Map
{
public:
    Map(int w, int h, int d, double res)
    : h(h), w(w), d(d), res(res), map(new SubvoxelMap*[h * w * d])
    {
        assert(res > 0.0);
    }

    ~Map()
    {
        for (int i = 0; i < h * w * d; ++i)
        {
            delete map[i];
        }

        delete[] map;

        fmt::print("Cleaned map\n");
    }

    const SubvoxelMap* at(double x, double y, double z) const
    {
        return map[util::to_arr_idx(x, y, z, res, w, d)];
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
            assert(map[util::to_arr_idx(x, y, z, res, w, d)] != nullptr);
        }

        Point submap_p = to_submap_point(x, y, z);
        fmt::print("{} {} {}, {}\n", x, y, z, util::to_arr_idx(x, y, z, res, w, d));
        auto* subvoxelmap = map[util::to_arr_idx(x, y, z, res, w, d)];
        assert(subvoxelmap != nullptr);
        subvoxelmap->at(submap_p.x, submap_p.y, submap_p.z) = val;
        fmt::print("Entry: {}\n", subvoxelmap->at(submap_p.x, submap_p.y, submap_p.z));
        fmt::print("Entry: {}\n", subvoxelmap->at(1, 1, 1));
    }

private:
    int h;
    int w;
    int d;
    double res;
    SubvoxelMap** map;

    void init_subvoxel_map(double x, double y, double z)
    {
        auto* submap = map[util::to_arr_idx(x, y, z, res, w, d)];
        assert(submap == nullptr);
        map[util::to_arr_idx(x, y, z, res, w, d)] = new SubvoxelMap(res, res, res, res / 10.0);
        assert(map[util::to_arr_idx(x, y, z, res, w, d)] != nullptr);
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