
/**
  * @file test.cpp
  * @author julian 
  * @date 12/2/21
 */
#include "local_map.h"
#include <catch2/catch_test_macros.hpp>

TEST_CASE("1d", "[1d]")
{
    int default_value = -1;
    LocalMap map(5, default_value);
    for (int i = 0; i < map.size_; ++i)
    {
        REQUIRE(map.value(i) == default_value);
    }
    
    // insert at coordinate 1 and 2
    map.insert(1, 3);
    map.insert(2, 4);
    REQUIRE(map.value(1) == 3);
    REQUIRE(map.value(2) == 4);
    
    for (int i = 0; i < map.size_; ++i)
    {
        if (i == map.pos_)
        {
            fmt::print("{:02}(p) ", map.data_[i]);
            continue;
        }
        fmt::print("{:02} ", map.data_[i]);
    }
    fmt::print("\n");
    
    map.shift(1);
    
    for (int i = 0; i < map.size_; ++i)
    {
        if (i == map.pos_)
        {
            fmt::print("{:02}(p) ", map.data_[i]);
            continue;
        }
        fmt::print("{:02} ", map.data_[i]);
    }
    fmt::print("\n");
    
    // after shift, same values
    REQUIRE(map.value(1) == 3);
    REQUIRE(map.value(2) == 4);
    
    map.shift(2);
    
    for (int i = 0; i < map.size_; ++i)
    {
        if (i == map.pos_)
        {
            fmt::print("{:02}(p) ", map.data_[i]);
            continue;
        }
        fmt::print("{:02} ", map.data_[i]);
    }
    fmt::print("\n");
    
    // after shift, same values
    REQUIRE(map.value(1) == 3);
    REQUIRE(map.value(2) == 4);
    
    map.shift(3);
    
    for (int i = offset; i < map.size_; ++i)
    {
        if (i == map.pos_)
        {
            fmt::print("{:02}(p) ", map.data_[i]);
            continue;
        }
        fmt::print("{:02} ", map.data_[i]);
    }
    fmt::print("\n");
    
    // after shift, same values
    REQUIRE(map.value(1) == 3);
    REQUIRE(map.value(2) == 4);
}
