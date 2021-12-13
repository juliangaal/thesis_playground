#pragma once

/**
  * @file GlobalMap.h
  * @author julian 
  * @date 12/3/21
 */

#include <vector>
#include <numeric>
#include <stdexcept>
#include "fmt/printf.h"
#include "../util/point.h"
#include "../subvoxelmap/util.h"

struct GlobalMap
{
    explicit GlobalMap(int size, int default_value, bool fill)
    : size_{size+1, size+1, size+1}
    , data_(size_.x() * size_.y() * size_.z())
    , offset_(size_/2)
    , default_val_(default_value)
    {
        if (fill)
        {
            std::iota(data_.begin(), data_.end(), -(size_.x() * size_.y() * size_.z())/2);
        }
        else
        {
            std::fill(data_.begin(), data_.end(), default_value);
        }
    }
    
    ~GlobalMap() = default;

    int at(Eigen::Vector3i p) const
    {
        return at(p.x(), p.y(), p.z());
    }

    int& at(Eigen::Vector3i p)
    {
        return at(p.x(), p.y(), p.z());
    }

    int at(int x, int y, int z) const
    {
        if (!in_bounds(x, y, z))
        {
            throw std::out_of_range(fmt::format("Global Map Out of Range! {}/{}/{}", x, y, z));
        }

        x += offset_.x();
        y += offset_.y();
        z += offset_.z();
        return data_[util::conv_3dindex_1dindex(x, y, z, size_.y(), size_.z())];
    }
    
    int& at(int x, int y, int z)
    {
        if (!in_bounds(x, y, z))
        {
            throw std::out_of_range(fmt::format("Global Map Out of Range! {}/{}/{}", x, y, z));
        }
        x += offset_.x();
        y += offset_.y();
        z += offset_.z();

        return data_[util::conv_3dindex_1dindex(x, y, z, size_.y(), size_.z())];
    }

    bool in_bounds(int x, int y, int z) const
    {
        return std::abs(x) < offset_.x() && std::abs(y) < offset_.y() && std::abs(z) < offset_.z();
    }

    bool in_bounds(const Eigen::Vector3i& index) const
    {
        auto p = index.cwiseAbs();
        return p.x() < offset_.x() && p.y() < offset_.y() && p.z() < offset_.z();
    }

    Eigen::Vector3i size_;
    std::vector<int> data_;
    Eigen::Vector3i offset_;
    int default_val_;
};