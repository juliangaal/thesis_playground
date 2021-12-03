#pragma once
/**
  * @file GlobalMap.h
  * @author julian 
  * @date 12/3/21
 */

#include <vector>
#include <numeric>
#include <stdexcept>

struct GlobalMap
{
    explicit GlobalMap(int size)
    : data_(size)
    , offset_(size/2)
    , size_(size)
    {
        std::iota(data_.begin(), data_.end(), -size/2);
    }
    
    ~GlobalMap() = default;
    
    int at(int index) const
    {
        if (index+offset_ > size_)
        {
            throw std::out_of_range("Global Map Out of Range!");
        }
        return data_[index+offset_];
    }
    
    int& at(int index)
    {
        if (index+offset_ > size_)
        {
            throw std::out_of_range("Global Map Out of Range!");
        }
        return data_[index+offset_];
    }
    
    std::vector<int> data_;
    int offset_;
    int size_;
};