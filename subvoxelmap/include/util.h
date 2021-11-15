#pragma once

#include <cmath>
#include <tuple>

namespace util
{

bool is_approx(double val1, double val2, double epsilon = 0.001)
{
    return std::abs(val1 - val2) < epsilon;
}

int conv3d21d(int x, int y, int z, int w, int d)
{
    return static_cast<int>(x + y * w + z * w * d);
}

std::tuple<int, int, int> to_arr_idx(double x, double y, double z, double res)
{
    return { static_cast<int>(x / res), static_cast<int>(y / res), static_cast<int>(z / res) };
}

int to_1d_arr_idx(double x, double y, double z, double res, int w, int d)
{
    int arrx, arry, arrz;
    std::tie(arrx, arry, arrz) = to_arr_idx(x, y, z, res);
    return static_cast<int>(conv3d21d(arrx, arry, arrz, w, d));
}

} // end namespace util
