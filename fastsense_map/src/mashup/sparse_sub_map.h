#pragma once

/**
  * @file SparseSubMap.h
  * @author julian 
  * @date 12/11/21
 */

#include "subvoxelmap/util.h"
#include "util/point.h"

struct SparseSubMap
{
    SparseSubMap(int size, int res, const int& default_val)
    : h_(size / res)
    , w_(size / res)
    , d_(size / res)
    , data_(new int[h_ * w_ * d_])
    , res_(res)
    , default_val_(default_val)
    , n_occupied_(0)
    {
        std::fill_n(data_, h_ * w_ * d_, default_val_);
    }
    
    ~SparseSubMap()
    {
        delete[] data_;
    }
    
    void insert_val(int x, int y, int z, int val)
    {
        data_[util::conv_3dpoint_1dindex(x, y, z, res_, w_, d_)] = val;
        n_occupied_++;
    }
    
    inline const int& at(int x, int y, int z) const
    {
        return data_[util::conv_3dpoint_1dindex(x, y, z, res_, w_, d_)];
    }
    
    void remove_val(int x, int y, int z)
    {
        data_[util::conv_3dpoint_1dindex(x, y, z, res_, w_, d_)] = default_val_;
        n_occupied_--;
    }
    
    inline bool empty() const
    {
        return n_occupied_ == 0;
    }
    
    inline int n_occupied() const
    {
        return n_occupied_;
    }
    
    inline int n_elems() const
    {
        return h_ * w_ * d_;
    }
    
    int h_;
    int w_;
    int d_;
    int* data_;
    int res_;
    const int& default_val_;
    int n_occupied_;
};