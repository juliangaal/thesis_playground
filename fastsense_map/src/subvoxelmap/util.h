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

template <typename T>
bool is_approx(T val1, T val2, T epsilon = 0.001)
{
    return std::abs(val1 - val2) < epsilon;
}

int conv_3dindex_1dindex(int x, int y, int z, int w, int d)
{
    return static_cast<int>(x + y * w + z * w * d);
}

template <typename T>
Point<int> conv_3dpoint_3dindex(T x, T y, T z, double res)
{
    return { static_cast<int>(x / res), static_cast<int>(y / res), static_cast<int>(z / res) };
}

template <typename T>
int conv_3dpoint_1dindex(T x, T y, T z, double res, int w, int d)
{
    Point<int> _3dindex = conv_3dpoint_3dindex(x, y, z, res);
    return conv_3dindex_1dindex(_3dindex.x, _3dindex.y, _3dindex.z, w, d);
}

} // end namespace util
