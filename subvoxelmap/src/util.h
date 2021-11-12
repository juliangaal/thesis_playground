#pragma once

namespace util
{

int conv3d21d(int x, int y, int z, int w, int d)
{
    return static_cast<int>(x + y * w + z * w * d);
}

int to_arr_idx(double x, double y, double z, double res, int w, int d)
{
    int arrx = static_cast<int>(x / res);
    int arry = static_cast<int>(y / res);
    int arrz = static_cast<int>(z / res);
    return static_cast<int>(conv3d21d(arrx, arry, arrz, w, d));
}

} // end namespace util
