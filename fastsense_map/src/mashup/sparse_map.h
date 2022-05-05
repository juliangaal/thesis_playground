#pragma once

/**
  * @file OccupiedChunk.h
  * @author julian 
  * @date 12/11/21
 */

#include "subvoxelmap/util.h"
#include "../util/point.h"

struct OccupiedChunk
{
    OccupiedChunk(int size, int res, const int& default_val)
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
    
    ~OccupiedChunk()
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

struct SparseMap
{
    SparseMap(unsigned int size, int res, int subvoxel_res, int default_val)
    : h_(static_cast<int>(size % 2 == 1 ? (size + 1) / res : (size / res)))
    , w_(h_)
    , d_(h_)
    , data_(new OccupiedChunk*[h_ * w_ * d_])
    , offset_(static_cast<int>(static_cast<int>(size % 2 == 1 ? (size + 1) / 2 : size / 2)))
    , res_(res)
    , subvoxel_res_(subvoxel_res)
    , default_val_(default_val)
    {
        std::fill_n(data_, h_ * w_ * d_, nullptr);
    }

    ~SparseMap() {
        for (int i = 0; i < h_ * w_ * d_; ++i)
        {
            delete data_[i];
        }
        delete[] data_;
    }

    void init_subvoxelmap(int index) const
    {
        if (subvoxel_unitialized(index)) {
            data_[index] = new OccupiedChunk(res_, subvoxel_res_, default_val_);
        }
    }

    inline bool subvoxel_unitialized(int x) const
    {
        return data_[x] == nullptr;
    }

    void insert(const Eigen::Vector3i& point, int val, bool apply_offset = false) const
    {
        insert(point.x(), point.y(), point.z(), val, apply_offset);
    }

    void insert(int x, int y, int z, int val, bool apply_offset = false) const
    {
        if (!in_bounds(x, y, z, apply_offset))
        {
            throw std::out_of_range(fmt::format("Out of range (voxelmap, insert): w/o offset {}, val: {}", x, val));
        }

        if (apply_offset)
        {
            x += offset_;
            y += offset_;
            z += offset_;
        }

        int index = util::conv_3dpoint_1dindex(x, y, z, res_, w_, d_);;
        init_subvoxelmap(index);
        insert_into_subvoxel(index, x, y, z, val);
    }

    const OccupiedChunk* sparse_submap(int x, int y, int z) const
    {
        if (!in_bounds(x, y, z, false))
        {
            throw std::out_of_range(fmt::format("Can't access subvoxelmap out of range (voxelmap, subvoxelmap): {}", x));
        }

        return sparse_submap_unchecked(x, y, z);
    }

    const OccupiedChunk* sparse_submap_unchecked(int x, int y, int z) const
    {
        return data_[util::conv_3dpoint_1dindex(x, y, z, res_, w_, d_)];
    }

    void insert_into_subvoxel(int index, int x, int y, int z, int val) const
    {
        // values here are always > 0 null, no need to specifically handle neg numbers and modulo
        auto sub_x = (x % res_) / subvoxel_res_;
        auto sub_y = (y % res_) / subvoxel_res_;
        auto sub_z = (z % res_) / subvoxel_res_;
        data_[index]->insert_val(sub_x, sub_y, sub_z, val);
    }

    const int& at(const Eigen::Vector3i& p, bool apply_offset = false) const
    {
        return at(p.x(), p.y(), p.z(), apply_offset);
    }

    const int& at(int x, int y, int z, bool apply_offset = false) const
    {
        if (!in_bounds(x, y, z, apply_offset))
        {
            throw std::out_of_range(fmt::format("Out of range (voxelmap, at): {}", x));
        }

        if (apply_offset)
        {
            x += offset_;
            y += offset_;
            z += offset_;
        }

        // offset already applied and in_bounds, so unchecked is safe
        auto* subvoxel = sparse_submap_unchecked(x, y, z);
        if (subvoxel == nullptr)
        {
            return default_val_;
        }

        // values here are always >0 null, no need to specifically handle neg numbers and modulo
        auto sub_x = (x % res_) / subvoxel_res_;
        auto sub_y = (y % res_) / subvoxel_res_;
        auto sub_z = (z % res_) / subvoxel_res_;
        return subvoxel->at(sub_x, sub_y, sub_z);
    }

    bool in_bounds(int x, int y, int z, bool offset_applied = false) const
    {
        if (offset_applied)
        {
            bool x_in_bounds = false;
            if (x < 0)
            {
                x_in_bounds = std::abs(x) <= offset_;
            }
            else
            {
                x_in_bounds = x < offset_;
            }

            bool y_in_bounds = false;
            if (y < 0)
            {
                y_in_bounds = std::abs(y) <= offset_;
            }
            else
            {
                y_in_bounds = y < offset_;
            }

            bool z_in_bounds = false;
            if (z < 0)
            {
                z_in_bounds = std::abs(z) <= offset_;
            }
            else
            {
                z_in_bounds = z < offset_;
            }

            return x_in_bounds && y_in_bounds && z_in_bounds;
        }

        return x < (h_ * res_) && y < (h_ * res_) && z < (h_ * res_);
    }

    int h_;
    int w_;
    int d_;
    OccupiedChunk** data_;
    int offset_;
    int res_;
    int subvoxel_res_;
    int default_val_;
};