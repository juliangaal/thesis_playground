#pragma once
/**
  * @file GlobalMap.h
  * @author julian 
  * @date 12/3/21
 */

#include <vector>
#include <numeric>
#include <stdexcept>
#include <fmt/printf.h>

struct GlobalMap
{
    explicit GlobalMap(int size)
    : data_(size+1)
    , offset_((size+1)/2)
    , size_(size+1)
    {
        std::iota(data_.begin(), data_.end(), -size/2);
    }
    
    ~GlobalMap() = default;
    
    int at(int index) const
    {
        if (!in_bounds(index))
        {
            throw std::out_of_range(fmt::format("Global Map Out of Range! {}", index));
        }
        return data_[index+offset_];
    }
    
    int& at(int index)
    {
        if (!in_bounds(index))
        {
            throw std::out_of_range(fmt::format("Global Map Out of Range! {}", index));
        }
        return data_[index+offset_];
    }

    bool in_bounds(int index) const
    {
        return std::abs(index) <= size_/2;
    }
    
    std::vector<int> data_;
    int offset_;
    int size_;
};