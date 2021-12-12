
/**
  * @file test.cpp
  * @author julian 
  * @date 12/11/21
 */

#include <catch2/catch_test_macros.hpp>
#include <fmt/printf.h>
#include "sparse_map.h"

TEST_CASE("SparseSubMap", "[SparseSubMap]")
{
    fmt::print("Testing SparseSubMap\n");
    {
        int size = 5;
        int default_val = -999;
        int res = 1;
        int occupied = 0;
        
        SparseSubMap map(size, res, default_val);
        
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
        
        SparseSubMap map(size, res, default_val);
        
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
                    REQUIRE(map.val_in_subvoxel(x, y, z, true) == default_val);
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
                    REQUIRE(map.val_in_subvoxel(x, y, z, true) == insert_val);
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
                    REQUIRE(map.val_in_subvoxel(x, y, z, true) == default_val);
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
                    REQUIRE(map.val_in_subvoxel(x, y, z, true) == insert_val);
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
                    REQUIRE(map.val_in_subvoxel(x, y, z, true) == default_val);
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
                    REQUIRE(map.val_in_subvoxel(x, y, z, true) == insert_val);
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