
/**
  * @file test.cpp
  * @author julian 
  * @date 12/11/21
 */

#include <catch2/catch_test_macros.hpp>
#include <fmt/printf.h>
#include "sparse_map.h"
#include "global_map.h"
#include "local_map.h"

TEST_CASE("OccupiedChunk", "[OccupiedChunk]")
{
    fmt::print("Testing OccupiedChunk\n");
    {
        int size = 5;
        int default_val = -999;
        int res = 1;
        int occupied = 0;
        
        OccupiedChunk map(size, res, default_val);
        
        REQUIRE(map.n_occupied() == 0);
        REQUIRE(map.n_elems() == std::pow(size / res, 3));
        
        for (int i = 0; i < size; ++i)
        {
            for (int j = 0; j < size; ++j)
            {
                for (int k = 0; k < size; ++k)
                {
                    map.insert_val(i, j, k, 5);
                    REQUIRE(map.at(i, j, k) == 5);
                    REQUIRE(map.n_occupied() == ++occupied);
                }
            }
        }
        
        for (int i = 0; i < size; ++i)
        {
            for (int j = 0; j < size; ++j)
            {
                for (int k = 0; k < size; ++k)
                {
                    map.remove_val(i, j, k);
                    REQUIRE(map.at(i, j, k) == default_val);
                    REQUIRE(map.n_occupied() == --occupied);
                }
            }
        }
        
        REQUIRE(map.n_occupied() == 0);
    }
    
    {
        int size = 10;
        int default_val = -999;
        int res = 2;
        int occupied = 0;
        
        OccupiedChunk map(size, res, default_val);
        
        REQUIRE(map.n_occupied() == 0);
        REQUIRE(map.n_elems() == std::pow(size / res, 3));
        
        for (int i = 0; i < size; ++i)
        {
            for (int j = 0; j < size; ++j)
            {
                for (int k = 0; k < size; ++k)
                {
                    map.insert_val(i, j, k, 5);
                    REQUIRE(map.at(i, j, k) == 5);
                    REQUIRE(map.n_occupied() == ++occupied);
                }
            }
        }
        
        for (int i = 0; i < size; ++i)
        {
            for (int j = 0; j < size; ++j)
            {
                for (int k = 0; k < size; ++k)
                {
                    map.remove_val(i, j, k);
                    REQUIRE(map.at(i, j, k) == default_val);
                    REQUIRE(map.n_occupied() == --occupied);
                }
            }
        }
        
        REQUIRE(map.n_occupied() == 0);
    }
}

TEST_CASE("VoxelMap", "[VoxelMap]")
{
    fmt::print("Testing SparseMap\n");

    {
        int default_val = -999;
        SparseMap map(10, 1, 1, default_val);
        REQUIRE(map.h_ == 10);
        REQUIRE(map.w_ == 10);
        REQUIRE(map.d_ == 10);
        REQUIRE(map.offset_ == 5);

        for (int i = 0; i < map.h_ * map.res_; ++i)
        {
            for (int j = 0; j < map.w_ * map.res_; ++j)
            {
                for (int k = 0; k < map.d_ * map.res_; ++k)
                {
                    int x = i - map.offset_;
                    int y = j - map.offset_;
                    int z = k - map.offset_;
                    REQUIRE(map.at(x, y, z, true) == default_val);
                }
            }
        }

        int insert_val = 1;
        for (int i = 0; i < map.h_ * map.res_; ++i)
        {
            for (int j = 0; j < map.w_ * map.res_; ++j)
            {
                for (int k = 0; k < map.d_ * map.res_; ++k)
                {
                    int x = i - map.offset_;
                    int y = j - map.offset_;
                    int z = k - map.offset_;
                    map.insert(x, y, z, insert_val, true);
                    REQUIRE(map.at(x, y, z, true) == insert_val);
                }
            }
        }

        REQUIRE_THROWS(map.insert(-6, 0, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, -6, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 0, -6, 1, true));
        REQUIRE_THROWS(map.insert(-8, 0, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, -8, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 0, -8, 1, true));
        REQUIRE_THROWS(map.insert(5, 0, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 5, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 0, 5, 1, true));
        REQUIRE_THROWS(map.insert(7, 0, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 7, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 0, 7, 1, true));
    }

    {
        int default_val = -999;
        SparseMap map(11, 2, 1, default_val);
        REQUIRE(map.h_ == 6);
        REQUIRE(map.w_ == 6);
        REQUIRE(map.d_ == 6);
        REQUIRE(map.offset_ == 6);

        for (int i = 0; i < map.h_ * map.res_; ++i)
        {
            for (int j = 0; j < map.w_ * map.res_; ++j)
            {
                for (int k = 0; k < map.d_ * map.res_; ++k)
                {
                    int x = i - map.offset_;
                    int y = j - map.offset_;
                    int z = k - map.offset_;
                    REQUIRE(map.at(x, y, z, true) == default_val);
                }
            }
        }

        int insert_val = 1;
        for (int i = 0; i < map.h_ * map.res_; ++i)
        {
            for (int j = 0; j < map.w_ * map.res_; ++j)
            {
                for (int k = 0; k < map.d_ * map.res_; ++k)
                {
                    int x = i - map.offset_;
                    int y = j - map.offset_;
                    int z = k - map.offset_;
                    map.insert(x, y, z, insert_val, true);
                    REQUIRE(map.at(x, y, z, true) == insert_val);
                }
            }
        }

        REQUIRE_THROWS(map.insert(-7, 0, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, -7, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 0, -7, 1, true));
        REQUIRE_THROWS(map.insert(-8, 0, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, -8, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 0, -8, 1, true));
        REQUIRE_THROWS(map.insert(6, 0, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 6, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 0, 6, 1, true));
        REQUIRE_THROWS(map.insert(7, 0, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 7, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 0, 7, 1, true));
    }

    {
        int default_val = -999;
        SparseMap map(12, 4, 1, default_val);
        REQUIRE(map.h_ == 3);
        REQUIRE(map.w_ == 3);
        REQUIRE(map.d_ == 3);
        REQUIRE(map.offset_ == 6);

        for (int i = 0; i < map.h_ * map.res_; ++i)
        {
            for (int j = 0; j < map.w_ * map.res_; ++j)
            {
                for (int k = 0; k < map.d_ * map.res_; ++k)
                {
                    int x = i - map.offset_;
                    int y = j - map.offset_;
                    int z = k - map.offset_;
                    REQUIRE(map.at(x, y, z, true) == default_val);
                }
            }
        }

        int insert_val = 1;
        for (int i = 0; i < map.h_ * map.res_; ++i)
        {
            for (int j = 0; j < map.w_ * map.res_; ++j)
            {
                for (int k = 0; k < map.d_ * map.res_; ++k)
                {
                    int x = i - map.offset_;
                    int y = j - map.offset_;
                    int z = k - map.offset_;
                    map.insert(x, y, z, insert_val, true);
                    REQUIRE(map.at(x, y, z, true) == insert_val);
                }
            }
        }

        REQUIRE_THROWS(map.insert(-7, 0, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, -7, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 0, -7, 1, true));
        REQUIRE_THROWS(map.insert(-8, 0, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, -8, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 0, -8, 1, true));
        REQUIRE_THROWS(map.insert(6, 0, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 6, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 0, 6, 1, true));
        REQUIRE_THROWS(map.insert(7, 0, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 7, 0, 1, true));
        REQUIRE_THROWS(map.insert(0, 0, 7, 1, true));
    }
}

TEST_CASE("Mashup", "[mashup]")
{
    fmt::print("Testing Mashup\n");

    int default_value = -999;
    GlobalMap global_map(10, 0, false);

    REQUIRE_THROWS(global_map.at(-6, 0, 0));
    REQUIRE_THROWS(global_map.at(6, 0, 0));

    LocalMap local_map(5, default_value, global_map);

    // local map represents local_map.size + 1 (if uneven) fields
    // (sub)voxelmap size is therefore this size / res
    // (sub)voxelmap offset represents offset from 0 from "real" size being represented (size * res)
//    REQUIRE(local_map.data_.size_ == 3);
//    REQUIRE(local_map.data_.size_ * local_map.data_.res_ == 6);
//    REQUIRE(local_map.data_.offset_ == 3);

    local_map.insert({ -2, 2, 0 }, 0);
    local_map.insert({ -1, 2, 0 }, 1);
    local_map.insert({ -2, 1, 0 }, 2);
    local_map.insert({ -1, 1, 0 }, 3);
    local_map.insert({ -2, 0, 0 }, 4);
    local_map.insert({ -1, 0, 0 }, 5);

    // test getter
    REQUIRE(local_map.get_pos() == Eigen::Vector3i(0, 0, 0));
    REQUIRE(local_map.get_size() == Eigen::Vector3i(5, 5, 5));
    REQUIRE(local_map.get_offset() == Eigen::Vector3i(2, 2, 2));

    // Test inserted values
    REQUIRE(local_map.value({0, 0, 0}) == default_value);
    REQUIRE(local_map.value({ -2, 2, 0 }) == 0);
    REQUIRE(local_map.value({ -1, 2, 0 }) == 1);
    REQUIRE(local_map.value({ -2, 1, 0 }) == 2);
    REQUIRE(local_map.value({ -1, 1, 0 }) == 3);
    REQUIRE(local_map.value({ -2, 0, 0 }) == 4);
    REQUIRE(local_map.value({ -1, 0, 0 }) == 5);
}