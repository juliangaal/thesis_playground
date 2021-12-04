
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
}
