#pragma once

/**
  * @file local_map.h
  * @author julian 
  * @date 12/2/21
 */
 

#include <fmt/printf.h>

struct LocalMap
{
    LocalMap(unsigned int size, int default_value)
        : size_{static_cast<int>(size % 2 == 1 ? size : size + 1)}
        , data_(new int[size_])
        , pos_(0)
        , offset_(size_ / 2)
        , default_value_(default_value)
    {
        std::fill_n(data_, size_, default_value_);
    }
    
    ~LocalMap()
    {
        delete[] data_;
    };
    
    inline const int& value(int x) const
    {
        if (!in_bounds(x))
        {
            throw std::out_of_range("Index out of bounds!");
        }
        
        return value_unchecked(x);
    }
    
    inline void insert(int x, int val) const
    {
        if (!in_bounds(x))
        {
            throw std::out_of_range("Index out of bounds!");
        }
    
        data_[get_index_of_point(x)] = val;
    }
    
    void shift(int new_pos)
    {
        int diff = new_pos - pos_;
        if (diff == 0)
        {
            return;
        }
        
        int start = pos_ - size_/ 2;
        int end = pos_ + size_/ 2;
        
        if (diff > 0)
        {
            end = start + diff - 1;
        }
        else
        {
            start = end + diff + 1;
        }
        
        fmt::print("--\nSHIFT\npos: {} offset: {} diff: {}\n", pos_, offset_, diff);
        fmt::print("start: {}, end: {}\n", start, end);
        
        pos_ += diff;
        offset_ = (offset_ + diff + size_) % size_;
        fmt::print("new pos: {}, new offset: {}\n--\n", pos_, offset_);
        
    }
    
    inline int get_index_of_point(int point) const
    {
        auto p = point - pos_ + offset_ + size_;
        return p % size_;
    }
    
    inline const int& value_unchecked(int point) const
    {
        return data_[get_index_of_point(point)];
    }
    
    inline bool in_bounds(int x) const
    {
        return x < size_;
    }
    
    /// length of local map, always odd so there is a middle cell
    int size_;

    /// actual data in map
    int* data_;
    
    /// center position
    int pos_;
    
    /// offset from data(0) to pos
    int offset_;
    
    int default_value_;
};
