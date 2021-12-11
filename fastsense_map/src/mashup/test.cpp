
/**
  * @file test.cpp
  * @author julian 
  * @date 12/11/21
 */

#include <catch2/catch_test_macros.hpp>
#include <fmt/printf.h>
#include "sparse_sub_map.h"

TEST_CASE("SparseSubMap", "[SparseSubMap]")
{
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