
/**
  * @file test.cpp
  * @author julian 
  * @date 12/2/21
 */
#include "local_map.h"
#include "voxel_map.h"

#include <fmt/ostream.h>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("1dVoxelMap", "[1dVoxelMap]")
{
    fmt::print("----\n1dVoxelMap\n----\n");

    int default_val = -999;
    VoxelMap1d map(10, 1, 1, default_val);
    REQUIRE(map.val_in_subvoxel(-5, true) == default_val);
    REQUIRE(map.val_in_subvoxel(-4, true) == default_val);
    REQUIRE(map.val_in_subvoxel(-3, true) == default_val);
    REQUIRE(map.val_in_subvoxel(-2, true) == default_val);
    REQUIRE(map.val_in_subvoxel(-1, true) == default_val);
    REQUIRE(map.val_in_subvoxel(0, true) == default_val);
    REQUIRE(map.val_in_subvoxel(1, true) == default_val);
    REQUIRE(map.val_in_subvoxel(2, true) == default_val);
    REQUIRE(map.val_in_subvoxel(3, true) == default_val);
    REQUIRE(map.val_in_subvoxel(4, true) == default_val);

    map.insert(4, 1, true);
    REQUIRE(map.val_in_subvoxel(4, true) == 1);

    fmt::print("voxelmap: {}", map);

    map.insert(2, 2, true);
    REQUIRE(map.val_in_subvoxel(2, true) == 2);

    fmt::print("voxelmap: {}", map);

    map.insert(-4, 1, true);
    REQUIRE(map.val_in_subvoxel(-4, true) == 1);

    fmt::print("voxelmap: {}", map);

    map.insert(0, 2, true);
    REQUIRE(map.val_in_subvoxel(0, true) == 2);

    fmt::print("voxelmap: {}", map);

    REQUIRE_THROWS(map.insert(-6, 1));
    REQUIRE_THROWS(map.insert(6, 1));

    fmt::print("voxelmap: {}", map);

    fmt::print("done\n");
}

TEST_CASE("1dLocalMap", "[1dLocalMap]")
{
    fmt::print("----\n1dLocalMap\n----\n");

    int default_value = -999;
    GlobalMap global_map(10, 0, false);
    fmt::print("initial global map: {}\n", fmt::join(global_map.data_, " , "));

    REQUIRE_THROWS(global_map.at(-6));
    REQUIRE_THROWS(global_map.at(6));

    LocalMap local_map(5, default_value, global_map);
    // in this scenario, local map is filled with values that already exist
//    local_map.fill();

    local_map.insert(-2, -2);
    local_map.insert(-1, -1);
//    local_map.insert(0, 0);
//    local_map.insert(1, 1);

    fmt::print("initial local map :           {}\n", local_map.data_);

    REQUIRE_THROWS(local_map.value(-3) == 3);
    REQUIRE(local_map.value(-2) == -2);
    REQUIRE(local_map.value(-1) == -1);
    REQUIRE(local_map.value(0) == default_value);
    REQUIRE(local_map.value(1) == default_value);
    REQUIRE(local_map.value(2) == default_value);
    REQUIRE_THROWS(local_map.value(3) == 3);

    local_map.shift(1); // new value will be loaded from 3

    REQUIRE_THROWS(local_map.value(-2) == 3);
    REQUIRE(local_map.value(-1) == -1);
    REQUIRE(local_map.value(0) == default_value);
    REQUIRE(local_map.value(1) == default_value);
    REQUIRE(local_map.value(2) == default_value);
    REQUIRE(local_map.value(3) == default_value);
    REQUIRE_THROWS(local_map.value(4) == 3);

    fmt::print("shift 1\n global map: {}\n local map:            {}\n", fmt::join(global_map.data_, " , "), local_map.data_);

    local_map.shift(2);

    REQUIRE_THROWS(local_map.value(-1) == -1);
    REQUIRE(local_map.value(0) == default_value);
    REQUIRE(local_map.value(1) == default_value);
    REQUIRE(local_map.value(2) == default_value);
    REQUIRE(local_map.value(3) == default_value);
    REQUIRE(local_map.value(4) == default_value);
    REQUIRE_THROWS(local_map.value(5) == -1);

    fmt::print("shift 2\n global map: {}\n local map:            {}\n", fmt::join(global_map.data_, " , "), local_map.data_);

    local_map.shift(3);

    REQUIRE_THROWS(local_map.value(0) == 0);
    REQUIRE(local_map.value(1) == default_value);
    REQUIRE(local_map.value(2) == default_value);
    REQUIRE(local_map.value(3) == default_value);
    REQUIRE(local_map.value(4) == default_value);
    REQUIRE(local_map.value(5) == default_value);
    REQUIRE_THROWS(local_map.value(6) == 0);

    fmt::print("shift 2\n global map: {}\n local map:            {}\n", fmt::join(global_map.data_, " , "), local_map.data_);
    fmt::print("done\n");
}

// Test in fastsense style
TEST_CASE("1dLocalMap_fastsense", "[1dLocalMap_fastsense]")
{
    GlobalMap global_map(100, 0, false);

    REQUIRE_THROWS(global_map.at(-101));
    REQUIRE_THROWS(global_map.at(101));

    int default_value = -999;
    LocalMap local_map(5, default_value, global_map);

    local_map.insert(-2, 0);
    local_map.insert(-1, 1);

    // Test initialization
    REQUIRE(local_map.get_pos() == 0);
    REQUIRE(local_map.get_size() == 5);
    REQUIRE(local_map.get_offset() == 2);

    // Test in_bounds
    REQUIRE(local_map.in_bounds(2));
    REQUIRE_FALSE(local_map.in_bounds(22));

    // test default values
    REQUIRE(local_map.value(0) == default_value);

    // test value access
    REQUIRE(local_map.value(-1) == 1);
    REQUIRE(local_map.value(-2) == 0);

    // test shift
    local_map.shift(5);
    REQUIRE(local_map.get_pos() == 5);
    REQUIRE(local_map.get_size() == 5);
    REQUIRE(local_map.get_offset() == 7 % 5);

    local_map.shift(10);
    REQUIRE(local_map.get_pos() == 10);
    REQUIRE(local_map.get_size() == 5);
    REQUIRE(local_map.get_offset() == 12 % 5);

    local_map.shift(15);
    REQUIRE(local_map.get_pos() == 15);
    REQUIRE(local_map.get_size() == 5);
    REQUIRE(local_map.get_offset() == 17 % 5);

    local_map.shift(20);
    REQUIRE(local_map.get_pos() == 20);
    REQUIRE(local_map.get_size() == 5);
    REQUIRE(local_map.get_offset() == 22 % 5);

    local_map.shift(24);
    REQUIRE(local_map.get_pos() == 24);
    REQUIRE(local_map.get_size() == 5);
    REQUIRE(local_map.get_offset() == 26 % 5);

    // Test in_bounds
    REQUIRE_FALSE(local_map.in_bounds(2));
    REQUIRE(local_map.in_bounds(22));

    // Test values
    CHECK(local_map.value(24) == default_value);
}
