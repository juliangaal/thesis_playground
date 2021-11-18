#pragma once

#include <cmath>
#include <tuple>

namespace util
{

template<typename T>
struct Point
{
    T x, y, z;
};

bool is_approx(double val1, double val2, double epsilon = 0.001)
{
    return std::abs(val1 - val2) < epsilon;
}

int conv_3dindex_1dindex(int x, int y, int z, int w, int d)
{
    return static_cast<int>(x + y * w + z * w * d);
}

Point<int> conv_3dpoint_3dindex(double x, double y, double z, double res)
{
    return { static_cast<int>(x / res), static_cast<int>(y / res), static_cast<int>(z / res) };
}

int conv_3dpoint_1dindex(double x, double y, double z, double res, int w, int d)
{
    Point<int> _3dindex = conv_3dpoint_3dindex(x, y, z, res);
    return conv_3dindex_1dindex(_3dindex.x, _3dindex.y, _3dindex.z, w, d);
}

} // end namespace util
