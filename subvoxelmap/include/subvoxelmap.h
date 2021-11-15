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
    SubvoxelMap(int w, int h, int d, double res)
    : map(new double[h * w * d]), w(w), h(h), d(d), res(res), size(w * h * d)
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

    void insert(double x, double y, double z, double val)
    {
        at(x, y, z) = val;
    }

    int _size() const
    {
        return size;
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

    double at_raw(int x, int y, int z)
    {
        int i = util::conv3d21d(x, y, z, w, d);
        if (i >= size)
        {
            delete[] map;
            throw std::runtime_error(fmt::format("Accessing subvoxelmap @ invalid location: accessed @ {} with size {}\n", i, size));
        }
        return map[i];
    }

    int w;
    int h;
    int d;
    double res;
    int size;
};

} // end namespace map
