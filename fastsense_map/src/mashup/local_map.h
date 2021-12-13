#pragma once

/**
  * @file local_map.h
  * @author julian 
  * @date 12/2/21
 */
 
#include "global_map.h"
#include "sparse_map.h"
#include "fmt/printf.h"
#include "../util/point.h"

struct LocalMap
{
    /**
     * LocalMap Constructor
     * @param size physical size represented, in meters
     * @param default_value default value for unitialized values in local map
     * @param map global map
     */
    LocalMap(unsigned int size, int default_value, GlobalMap& map)
    : size_{static_cast<int>(size % 2 == 1 ? size : size + 1),
            static_cast<int>(size % 2 == 1 ? size : size + 1),
            static_cast<int>(size % 2 == 1 ? size : size + 1)},
      data_{size, 1, 1, -999},
      pos_{Eigen::Vector3i::Zero()},
      offset_{size_ / 2},
      global_map_{map}
    {
    }

    /// Destructor
    ~LocalMap() = default;

    /// return value at global coordinate x from correct index in local map/(sub)voxelmap
    inline const int& value(const Eigen::Vector3i& point) const
    {
        if (!in_bounds(point))
        {
            throw std::out_of_range("Index out of bounds!");
        }
        
        return value_unchecked(point);
    }

    /// insert value into local map/(sub)voxelmap
    inline void insert(const Eigen::Vector3i& point, int val) const
    {
        if (!in_bounds(point))
        {
            throw std::out_of_range("Index out of bounds!");
        }

        data_.insert(get_index_of_point(point), val);
    }

    /// fill local map with corresponding values in global map
    void fill()
    {
        for (int i = pos_.x() - offset_.x(); i < pos_.x() + offset_.x() + 1; ++i)
        {
            for (int j = pos_.y() - offset_.y(); j < pos_.y() + offset_.y() + 1; ++j)
            {
                for (int k = pos_.z() - offset_.z(); k < pos_.z() + offset_.z() + 1; ++k)
                {
//                    data_.insert(i, j, k, global_map_.at(i, j, k), true);
                }
            }
        }
    }

    /**
     * Shift: shift consists of
     * - saving values that fall out of local map in global map
     * - adapting pos and offset to new pos
     * - loading values from global map that are now in local map window
     * @param new_pos
     */
    void shift(const Eigen::Vector3i& new_pos)
    {
        Eigen::Vector3i diff = new_pos - pos_;

        // TODO is this appropriate behavior?
        assert(std::abs(diff.x()) <= size_.x() &&
               std::abs(diff.y()) <= size_.y() &&
               std::abs(diff.z()) <= size_.z());

        for (int axis = 0; axis < 3; axis++) {
            if (diff[axis] == 0) {
                return;
            }

            Eigen::Vector3i start = pos_ - size_ / 2;
            Eigen::Vector3i end = pos_ + size_ / 2;

            if (diff[axis] > 0) // shift happened to right
            {
                end[axis] = start[axis] + diff[axis] - 1;
            } else {
                start[axis] = end[axis] + diff[axis] + 1;
            }
//            save_area(start, end);

            pos_[axis] += diff[axis];
            offset_[axis] = (offset_[axis] + diff[axis] + size_[axis]) % size_[axis];

            // Step #3: detect area to load
            // it is determined by pos and the amount of shift (diff)
            start = pos_ - size_ / 2;
            end = pos_ + size_ / 2;

            // if shift happens to right, load end until end - (diff - 1)
            if (diff[axis] > 0) {
                start[axis] = end[axis] - (diff[axis] - 1);
            }
                // if shift happens to the left, load start until start + (abs(diff) - 1)
            else {
                // end = start + abs(diff) - 1,  but diff < 0
                end[axis] = start[axis] - diff[axis] - 1;
            }
//            load_area(start, end);
        }
    }

    /// load area from coordinate start to end from global map
    /// and insert into subvoxelmap
    void load_area(int start, int end)
    {
//        fmt::print(" loading: ");
//        for (int i = start; i < end + 1; ++i)
//        {
//            fmt::print("{} ", i);
//            int global_map_val = global_map_.at(i);
//            if (global_map_val != global_map_.default_val_)
//            {
////                insert(i, global_map_.at(i));
//            }
//        }
//        fmt::print("\n");
    }

    /// save area from coordinate start to end in global map
    void save_area(int start, int end)
    {
//        fmt::print(" saving: ");
//        for (int i = start; i < end + 1; ++i)
//        {
//            fmt::print("{} ", i);
//            const auto& save_val = value(i);
//            if (save_val != data_.default_val_)
//            {
//                global_map_.at(i) = save_val;
//            }
//            data_.delete_from_subvoxel(get_index_of_point(i));
//            data_.cleanup_after_save(get_index_of_point(i));
//        }
//        fmt::print("\n");
    }

    /// convert from global coordinate to local map index
    inline Eigen::Vector3i get_index_of_point(const Eigen::Vector3i& point) const
    {
        auto p = point - pos_ + offset_ + size_;
        return { p.x() % size_.x(), p.y() % size_.y(),p.z() % size_.z() };
    }

    /// return value from (sub)voxelmap
    inline const int& value_unchecked(const Eigen::Vector3i& point) const
    {
        return data_.at(get_index_of_point(point));
    }

    /// check if coordinate is in_bounds of local map
    inline bool in_bounds(const Eigen::Vector3i& x) const
    {
        auto p = (x - pos_).cwiseAbs();
        return p.x() <= size_.x() / 2 && p.y() <= size_.y() / 2 && p.z() <= size_.z() / 2;
    }

    /// return pos
    inline Eigen::Vector3i get_pos() const
    {
        return pos_;
    }

    /// return size
    inline Eigen::Vector3i get_size() const
    {
        return size_;
    }

    /// return offset
    inline Eigen::Vector3i get_offset() const
    {
        return offset_;
    }
    
    /// length of local map, always odd so there is a middle cell
    Eigen::Vector3i size_;

    /// actual data in map
    SparseMap data_;
    
    /// center position
    Eigen::Vector3i pos_;
    
    /// offset from data(0) to pos
    Eigen::Vector3i offset_;
    
    GlobalMap& global_map_;
};
