#pragma once

/**
  * @file voxel_map.h
  * @author julian
  * @date 12/3/21
 */

#include <numeric>
#include <cassert>

struct SubVoxelMap1d
{
    SubVoxelMap1d(int size, int res)
    : data_(new int[size / res])
    , size_(size / res)
    , res_(res)
    {
        std::fill_n(data_, size_, -999);
    }

    ~SubVoxelMap1d()
    {
        delete[] data_;
    }

    void insert_val_at_index(int x, int val)
    {
        data_[x] = val;
    }

    int* data_;
    int size_;
    int res_;
};

struct VoxelMap1d
{
    VoxelMap1d(unsigned int size, int res, int subvoxel_res)
    : size_(static_cast<int>(size % 2 == 1 ? size : size + 1))
    , data_(new SubVoxelMap1d*[size])
    , offset_(size/2)
    , res_(res)
    , subvoxel_res_(subvoxel_res)
    {
        std::fill_n(data_, size_, nullptr);
    }

    ~VoxelMap1d()
    {
        for (int i = 0; i < size_; ++i)
        {
            delete data_[i];
        }
        delete[] data_;
    }

    void init_subvoxelmap(int index) {
        data_[index] = new SubVoxelMap1d(res_, subvoxel_res_);
    }

    void insert(int x, int val)
    {
        if (!in_bounds(x))
        {
            throw std::out_of_range(fmt::format("Out of range (voxelmap, insert): {}/{}", x - offset_, x));
        }

        x += offset_;

        auto voxelmap_index = x / res_;
        if (subvoxel_unitialized(voxelmap_index))
        {
            init_subvoxelmap(voxelmap_index);
        }

        insert_subvoxel(x, voxelmap_index, val);
    }

    void insert_subvoxel(int x, int voxelmap_index, int val)
    {
        // TODO build into original implemenation
        auto x_in_subvoxel = (x % res_) / subvoxel_res_;
        data_[voxelmap_index]->insert_val_at_index(x_in_subvoxel, val);
    }

    int val_in_subvoxel(int x) const
    {
        if (!in_bounds(x))
        {
            throw std::out_of_range(fmt::format("Out of range (voxelmap, val_in_subvoxel): {}/{}", x - offset_, x));
        }

        x += offset_;

        auto voxelmap_index = x / res_;
        auto* subvoxel = data_[voxelmap_index];

        if (subvoxel == nullptr)
        {
            return -999;
        }

        auto x_in_subvoxel = (x % res_) / subvoxel_res_;
        return subvoxel->data_[x_in_subvoxel];
    }

    bool subvoxel_unitialized(int x) const
    {
        return data_[x] == nullptr;
    }


    bool in_bounds(int x) const
    {
        return std::abs(x) <= offset_;
    }

    int size_;
    SubVoxelMap1d** data_;
    int offset_;
    int res_;
    int subvoxel_res_;
};