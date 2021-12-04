
/**
  * @file test.cpp
  * @author julian 
  * @date 12/2/21
 */
#include "local_map.h"
#include "voxel_map.h"
#include <catch2/catch_test_macros.hpp>

TEST_CASE("1dVoxelMap", "[1dVoxelMap]")
{
    fmt::print("----\n1dVoxelMap\n----\n");

    VoxelMap1d map(10, 1, 1);
    REQUIRE(map.val_in_subvoxel(-5) == -999);
    REQUIRE(map.val_in_subvoxel(-4) == -999);
    REQUIRE(map.val_in_subvoxel(-3) == -999);
    REQUIRE(map.val_in_subvoxel(-2) == -999);
    REQUIRE(map.val_in_subvoxel(-1) == -999);
    REQUIRE(map.val_in_subvoxel(0) == -999);
    REQUIRE(map.val_in_subvoxel(1) == -999);
    REQUIRE(map.val_in_subvoxel(2) == -999);
    REQUIRE(map.val_in_subvoxel(3) == -999);
    REQUIRE(map.val_in_subvoxel(4) == -999);

    map.insert(4, 1);
    REQUIRE(map.val_in_subvoxel(4) == 1);

    map.insert(4, 2);
    REQUIRE(map.val_in_subvoxel(4) == 2);

    map.insert(-4, 1);
    REQUIRE(map.val_in_subvoxel(-4) == 1);

    map.insert(-4, 2);
    REQUIRE(map.val_in_subvoxel(-4) == 2);

    REQUIRE_THROWS(map.insert(-6, 1));
    REQUIRE_THROWS(map.insert(6, 1));

    fmt::print("done\n");
}

TEST_CASE("1dLocalMap", "[1dLocalMap]")
{
    fmt::print("----\n1dLocalMap\n----\n");

    int default_value = -1;
    GlobalMap global_map(10);
    fmt::print("initial global map: {}\n", fmt::join(global_map.data_, ", "));

    REQUIRE_THROWS(global_map.at(-6));
    REQUIRE_THROWS(global_map.at(6));

    LocalMap local_map(5, default_value, global_map);
    // in this scenario, local map is filled with values that already exist
    local_map.fill();

    fmt::print("initial local map : {}\n", fmt::join(local_map.data_, ", "));

    REQUIRE_THROWS(local_map.value(-3) == 3);
    REQUIRE(local_map.value(-2) == -2);
    REQUIRE(local_map.value(-1) == -1);
    REQUIRE(local_map.value(0) == 0);
    REQUIRE(local_map.value(1) == 1);
    REQUIRE(local_map.value(2) == 2);
    REQUIRE_THROWS(local_map.value(3) == 3);

    local_map.shift(1);

    REQUIRE_THROWS(local_map.value(-2) == 3);
    REQUIRE(local_map.value(-1) == -1);
    REQUIRE(local_map.value(0) == 0);
    REQUIRE(local_map.value(1) == 1);
    REQUIRE(local_map.value(2) == 2);
    REQUIRE(local_map.value(3) == 3);
    REQUIRE_THROWS(local_map.value(4) == 3);

    fmt::print("shift 1\n global map: {}\n local map: {}\n", fmt::join(global_map.data_, ", "), fmt::join(local_map.data_, ", "));
    
    local_map.shift(2);

    REQUIRE_THROWS(local_map.value(-1) == -1);
    REQUIRE(local_map.value(0) == 0);
    REQUIRE(local_map.value(1) == 1);
    REQUIRE(local_map.value(2) == 2);
    REQUIRE(local_map.value(3) == 3);
    REQUIRE(local_map.value(4) == 4);
    REQUIRE_THROWS(local_map.value(5) == -1);

    fmt::print("shift 2\n global map: {}\n local map: {}\n", fmt::join(global_map.data_, ", "), fmt::join(local_map.data_, ", "));
    
    local_map.shift(3);

    REQUIRE_THROWS(local_map.value(0) == 0);
    REQUIRE(local_map.value(1) == 1);
    REQUIRE(local_map.value(2) == 2);
    REQUIRE(local_map.value(3) == 3);
    REQUIRE(local_map.value(4) == 4);
    REQUIRE(local_map.value(5) == 5);
    REQUIRE_THROWS(local_map.value(6) == 0);

    fmt::print("shift 2\n global map: {}\n local map: {}\n", fmt::join(global_map.data_, ", "), fmt::join(local_map.data_, ", "));
    fmt::print("done\n");
}
