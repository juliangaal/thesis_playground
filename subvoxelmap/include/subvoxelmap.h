#pragma once

#include <fmt/printf.h>
#include <cassert>

namespace map
{

struct Point
{
    double x, y, z;
};

class SubvoxelMap
{
public:
    SubvoxelMap(int w, int h, int d, double res, int map_size)
    : map(new double[h * w * d]), w(w), h(h), d(d), res(res), size(w * h * d), map_size(map_size)
    {
        assert(w > res);
        std::memset(map, 10, w * h * d * sizeof(double));
    }

    ~SubvoxelMap()
    {
        delete[] map;
    }

    SubvoxelMap& operator=(const SubvoxelMap&) = delete;
    SubvoxelMap& operator=(SubvoxelMap&&) = delete;
    SubvoxelMap(const SubvoxelMap&) = delete;
    SubvoxelMap(SubvoxelMap&&) = delete;

    void insert()
    {

    }

    double at(int x, int y, int z) const
    {
        if (!in_range(x, y, z))
        {
            delete[] map;
            throw std::runtime_error(fmt::format("Accessing subvoxelmap @ invalid location: accessed @ ({}/{}/{}) with size {}\n", x, y, z, size));
        }
        int i = util::conv3d21d(x, y, z, w, d);
        return map[i];
    }

    void insert(double x, double y, double z, double val)
    {
        at(x, y, z) = val;
    }

    int _size() const
    {
        return size;
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

    double *map;

private:
    double& at(double x, double y, double z)
    {
        int i = util::to_1d_arr_idx(x, y, z, res, w, d);
        if (i >= size)
        {
            delete[] map;
            throw std::runtime_error(fmt::format("Accessing subvoxelmap @ invalid location: accessed @ {} with size {}\n", i, size));
        }
        return map[i];
    }

    bool in_range(int x, int y, int z) const
    {
        return x >= 0 && x < w && y >= 0 && y < h && z >= 0 && z < d;
    }

    int w;
    int h;
    int d;
    double res;
    int size;
    int map_size;
};

} // end namespace map
