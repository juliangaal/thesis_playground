
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
    GlobalMap global_map(10);
    fmt::print("initial global map: {}\n", fmt::join(global_map.data_, ", "));
    
    LocalMap local_map(5, default_value, global_map);
    
    fmt::print("initial local map : {}\n", fmt::join(local_map.data_, ", "));
    
    REQUIRE(local_map.value(-2) == -2);
    REQUIRE(local_map.value(-1) == -1);
    REQUIRE(local_map.value(0) == 0);
    REQUIRE(local_map.value(1) == 1);
    REQUIRE(local_map.value(2) == 2);
    
    local_map.shift(1);
    
    fmt::print("shift 1\n global map: {}\n local map: {}\n", fmt::join(global_map.data_, ", "), fmt::join(local_map.data_, ", "));
    
    local_map.shift(2);
    
    fmt::print("shift 2\n global map: {}\n local map: {}\n", fmt::join(global_map.data_, ", "), fmt::join(local_map.data_, ", "));
    
//
//    // after shift, same values
//    REQUIRE(local_map.value(1) == 3);
////    REQUIRE(map.value(2) == 4);
//
//    local_map.shift(3);
//
//    for (int i = 0; i < local_map.size_; ++i)
//    {
//        if (i == local_map.offset_)
//        {
//            fmt::print("{:02}(p) ", local_map.data_[i]);
//            continue;
//        }
//        fmt::print("{:02} ", local_map.data_[i]);
//    }
//    fmt::print("\n");
//
//    // after shift, same values
//    REQUIRE(local_map.value(1) == 3);
////    REQUIRE(map.value(2) == 4);
//
////    map.shift(3);
////
////    for (int i = 0; i < map.size_; ++i)
////    {
////        if (i == map.offset_)
////        {
////            fmt::print("{:02}(p) ", map.data_[i]);
////            continue;
////        }
////        fmt::print("{:02} ", map.data_[i]);
////    }
////    fmt::print("\n");
////
//    // after shift, same values
//    REQUIRE(local_map.value(1) == 3);
////    REQUIRE(map.value(2) == 4);
}
