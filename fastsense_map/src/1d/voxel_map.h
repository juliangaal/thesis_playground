#pragma once

/**
  * @file voxel_map.h
  * @author julian
  * @date 12/3/21
 */

#include <numeric>
#include <cassert>
#include <iomanip>

/*
 * Changes done to integrate into local map:
 * - offset is already applied in local map, make optional
 *
 */

struct SubVoxelMap1d
{
    SubVoxelMap1d(int size, int res, const int& default_val)
    : data_(new int[size / res])
    , size_(size / res)
    , res_(res)
    , default_val_(default_val)
    , occupied_(0)
    {
        std::fill_n(data_, size_, default_val_);
    }

    ~SubVoxelMap1d()
    {
        delete[] data_;
    }

    void insert_val_at_index(int x, int val)
    {
        data_[x] = val;
        occupied_++;
    }

    void remove_val_at_index(int x)
    {
        data_[x] = default_val_;
        occupied_--;
    }

    bool empty() const
    {
        return occupied_ == 0;
    }

    int* data_;
    int size_;
    int res_;
    const int& default_val_;
    int occupied_;
};

struct VoxelMap1d
{
    VoxelMap1d(unsigned int size, int res, int subvoxel_res, int default_val)
    : size_(static_cast<int>(size % 2 == 1 ? size : size + 1))
    , data_(new SubVoxelMap1d*[size])
    , offset_(size/2)
    , res_(res)
    , subvoxel_res_(subvoxel_res)
    , default_val_(default_val)
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

    void init_subvoxelmap(int index)
    {
        if (subvoxel_unitialized(index)) {
            data_[index] = new SubVoxelMap1d(res_, subvoxel_res_, default_val_);
        }
    }

    void destroy_subvoxelmap(int index)
    {
        delete data_[index];
        data_[index] = nullptr;
    }

    void cleanup_after_save(int index)
    {
        auto voxelmap_index = index / res_;
        auto* subvoxel = data_[voxelmap_index];
        if (subvoxel != nullptr)
        {
            if (subvoxel->empty())
            {
                destroy_subvoxelmap(voxelmap_index);
            }
        }
    }

    void insert(int x, int val, bool apply_offset = false)
    {
        if (!in_bounds(x, apply_offset))
        {
            throw std::out_of_range(fmt::format("Out of range (voxelmap, insert): w/o offset {}, val: {}", x, val));
        }

        if (apply_offset)
        {
            x += offset_;
        }

        auto voxelmap_index = x / res_;
        init_subvoxelmap(voxelmap_index);
        insert_into_subvoxel(x, voxelmap_index, val);
    }

    void delete_from_subvoxel(int x)
    {
        auto voxelmap_index = x / res_;
        auto* subvoxel = data_[voxelmap_index];
        if (subvoxel != nullptr)
        {
            auto x_in_subvoxel = (x % res_) / subvoxel_res_;
            data_[voxelmap_index]->remove_val_at_index(x_in_subvoxel);
        }
    }

    void insert_into_subvoxel(int x, int voxelmap_index, int val)
    {
        // TODO build into original implemenation
        auto x_in_subvoxel = (x % res_) / subvoxel_res_;
        data_[voxelmap_index]->insert_val_at_index(x_in_subvoxel, val);
    }

    SubVoxelMap1d* subvoxelmap(int x)
    {
        if (!in_bounds(x, false))
        {
            throw std::out_of_range(fmt::format("Can't access subvoxelmap out of range (voxelmap, subvoxelmap): {}", x));
        }

        auto voxelmap_index = x / res_;
        return data_[voxelmap_index];
    }

    const int& val_in_subvoxel(int x, bool apply_offset = false) const
    {
        if (!in_bounds(x, apply_offset))
        {
            throw std::out_of_range(fmt::format("Out of range (voxelmap, val_in_subvoxel): {}", x));
        }

        if (apply_offset)
        {
            x += offset_;
        }

        auto voxelmap_index = x / res_;
        auto* subvoxel = data_[voxelmap_index];

        if (subvoxel == nullptr)
        {
            return default_val_;
        }

        auto x_in_subvoxel = (x % res_) / subvoxel_res_;
        return subvoxel->data_[x_in_subvoxel];
    }

    bool subvoxel_unitialized(int x) const
    {
        return data_[x] == nullptr;
    }


    bool in_bounds(int x, bool offset_applied = false) const
    {
        if (offset_applied)
        {
            return std::abs(x) <= offset_;
        }

        return x < (size_ * res_);
    }

    friend std::ostream& operator<<(std::ostream& os, const VoxelMap1d& map)
    {
        for (int x = 0; x < map.size_; ++x)
        {
            auto voxelmap_index = x / map.res_;
            auto* subvoxel = map.data_[voxelmap_index];
            if (x % map.res_ == 0)
            {
                os << "|";
            }

            if (subvoxel == nullptr)
            {
                    os << " - ";
            }
            else
            {
                for (int i = 0; i < subvoxel->size_; ++i)
                {
                    os << std::setw(2) << subvoxel->data_[i] << " ";
                }
            }
        }
        os << "|\n";
    }

    int size_;
    SubVoxelMap1d** data_;
    int offset_;
    int res_;
    int subvoxel_res_;
    int default_val_;
};