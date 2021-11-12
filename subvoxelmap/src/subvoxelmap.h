#pragma once

#include <fmt/printf.h>

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
    : w(w), h(h), d(d), res(res), map(new double[h * w * d]{std::numeric_limits<double>::quiet_NaN()})
    {
    }

    ~SubvoxelMap()
    {
        delete[] map;
        fmt::print("Cleaned up subvoxelmap\n");
    }

    double &at(double x, double y, double z)
    {
        return map[util::to_arr_idx(x, y, z, res, w, d)];
    }

    double &at(Point p)
    {
        return at(p.x, p.y, p.z);
    }

    double *map;


private:
    int w;
    int h;
    int d;
    double res;
};

} // end namespace map
