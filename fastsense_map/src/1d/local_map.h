#pragma once

/**
  * @file local_map.h
  * @author julian 
  * @date 12/2/21
 */
 
#include "global_map.h"
#include "voxel_map.h"
#include <fmt/printf.h>

struct LocalMap
{
    LocalMap(unsigned int size, int default_value, GlobalMap& map)
        : size_{static_cast<int>(size % 2 == 1 ? size : size + 1)}
        , data_(size_, 1, 1, default_value)
        , pos_(0)
        , offset_(size_ / 2)
        , global_map_(map)
    {

    }
    
    ~LocalMap() = default;
    
    inline const int& value(int x) const
    {
        if (!in_bounds(x))
        {
            throw std::out_of_range("Index out of bounds!");
        }
        
        return value_unchecked(x);
    }
    
    inline void insert(int x, int val)
    {
        if (!in_bounds(x))
        {
            throw std::out_of_range("Index out of bounds!");
        }

        data_.insert(get_index_of_point(x), val);
    }

    void fill()
    {
        for (int i = pos_ - offset_; i < pos_ + offset_ + 1; ++i)
        {
            data_.insert(i, global_map_.at(i), true);
        }
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
        
        if (diff > 0) // shift happened to right
        {
            end = start + diff - 1;
        }
        else
        {
            start = end + diff + 1;
        }
        save_area(start, end);
    
        pos_ += diff;
        offset_ = (offset_ + diff + size_) % size_;

        // Step #3: detect area to load
        // it is determined by pos and the amount of shift (diff)
        start = pos_ - size_ / 2;
        end = pos_ + size_ / 2;
        
        // if shift happens to right, load end until end - (diff - 1)
        if (diff > 0)
        {
            start = end - (diff - 1);
        }
        // if shift happens to the left, load start until start + (abs(diff) - 1)
        else
        {
            // end = start + abs(diff) - 1,  but diff < 0
            end = start - diff - 1;
        }
        load_area(start, end);
    }
    
    void load_area(int start, int end)
    {
        fmt::print(" loading: ");
        for (int i = start; i < end + 1; ++i)
        {
            fmt::print("{} ", i);
            int global_map_val = global_map_.at(i);
            if (global_map_val != global_map_.default_val_)
            {
                insert(i, global_map_.at(i));
            }
        }
        fmt::print("\n");
    }
    
    void save_area(int start, int end)
    {
        fmt::print(" saving: ");
        for (int i = start; i < end + 1; ++i)
        {
            fmt::print("{} ", i);
            global_map_.at(i) = value(i);
            data_.delete_from_subvoxel(get_index_of_point(i));
            data_.cleanup_after_save(get_index_of_point(i));
        }
        fmt::print("\n");
    }
    
    inline int get_index_of_point(int point) const
    {
        auto p = point - pos_ + offset_ + size_;
        return p % size_;
    }
    
    inline const int& value_unchecked(int point) const
    {
        return data_.val_in_subvoxel(get_index_of_point(point));
    }
    
    inline bool in_bounds(int x) const
    {
        return std::abs(x - pos_) <= size_ / 2;
    }
    
    /// length of local map, always odd so there is a middle cell
    int size_;

    /// actual data in map
    VoxelMap1d data_;
    
    /// center position
    int pos_;
    
    /// offset from data(0) to pos
    int offset_;
    
    GlobalMap& global_map_;
};
