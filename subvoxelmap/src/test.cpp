#include "util.h"
#include "map.h"
#include "parameters.h"

#include <iostream>
#include <cmath>
#include <numeric>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Utils", "[utils]")
{
    double nan = std::numeric_limits<double>::quiet_NaN();
    REQUIRE(std::isnan(nan));
}

TEST_CASE("Subvoxelamp", "[subvoxelmap]")
{
    double epsilon = 0.0001;
    double res = 1.0;
    int map_size = 4;
    map::SubvoxelMap map(4, 1);

    // Test Initialization
    REQUIRE(map._res() == res);
    REQUIRE(map._size() == map._h() * map._w() * map._d());
    REQUIRE(map._map_size() == map_size);
    REQUIRE(map._h() == static_cast<int>(map_size / res));
    REQUIRE(map._w() == static_cast<int>(map_size / res));
    REQUIRE(map._d() == static_cast<int>(map_size / res));

    for (int x = 0; x < map._h(); ++x)
    {
        for (int y = 0; y < map._w(); ++y)
        {
            for (int z = 0; z < map._d(); ++z)
            {
                REQUIRE(std::isnan(map.at_index(x, y, z)));
            }
        }
    }

    // Test at limits (int)
    REQUIRE_THROWS(map.at_index(-1, 0, 0));
    REQUIRE_THROWS(map.at_index(0, -1, 0));
    REQUIRE_THROWS(map.at_index(0, 0, -1));
    REQUIRE_THROWS(map.at_index(map_size, 0, 0));
    REQUIRE_THROWS(map.at_index(0, map_size, 0));
    REQUIRE_THROWS(map.at_index(0, 0, map_size));

    // Test at limits (double)
    REQUIRE_THROWS(map.at_point(-1.0, 0.0, 0.0));
    REQUIRE_THROWS(map.at_point(0.0, -1.0, 0.0));
    REQUIRE_THROWS(map.at_point(0.0, 0.0, -1.0));
    REQUIRE_THROWS(map.at_point(static_cast<double>(map_size), 0.0, 0.0));
    REQUIRE_THROWS(map.at_point(0.0, static_cast<double>(map_size), 0.0));
    REQUIRE_THROWS(map.at_point(0.0, 0.0, static_cast<double>(map_size)));

    // Test insert and borders of inserted cube
    // insert at (0, 0, 0)
    map.insert(0, 0, 0, 3.0);
    REQUIRE(map.at_index(0, 0, 0) == 3.0);
    REQUIRE(map.at_point(0.0, 0.0, 0.0) == 3.0);
    REQUIRE(std::isnan(map.at_point(res, res, res)));
    REQUIRE(map.at_point(res-epsilon, res-epsilon, res-epsilon) == 3);

    REQUIRE_THROWS(map.insert(-1, 0, 0, 8));
    REQUIRE_THROWS(map.insert(0, -1, 0, 8));
    REQUIRE_THROWS(map.insert(0, 0, -1, 8));
    REQUIRE_THROWS(map.insert(map_size, 0, 0, 8));
    REQUIRE_THROWS(map.insert(0, map_size, 0, 8));
    REQUIRE_THROWS(map.insert(0, 0, map_size, 8));

    map.insert(1, 1, 1, 6.0);
    REQUIRE(map.at_index(1, 1, 1) == 6.0);
    REQUIRE(map.at_point(1.0, 1.0, 1.0) == 6.0);
    REQUIRE(std::isnan(map.at_point(1.0 + res, 1.0 + res, 1.0 + res)));
    REQUIRE(map.at_point(1.0 + res -epsilon, 1.0 + res-epsilon, 1.0 + res - epsilon) == 6.0);

    // Test clearing
    map.clear();
    for (int x = 0; x < map._h(); ++x)
    {
        for (int y = 0; y < map._w(); ++y)
        {
            for (int z = 0; z < map._d(); ++z)
            {
                REQUIRE(std::isnan(map.at_index(x, y, z)));
            }
        }
    }

    // Test second constructor
    map::SubvoxelMap map2(10, 10, 10, 0.1, 1);
    REQUIRE(map2._res() == 0.1);
    REQUIRE(map2._size() == map2._h() * map2._w() * map2._d());
    REQUIRE(map2._map_size() == 1);
    REQUIRE(map2._h() == 10);
    REQUIRE(map2._w() == 10);
    REQUIRE(map2._d() == 10);
}

void test_map(map::Map& map, int map_size, double map_res, double subvoxel_res)
{
    // Test Initialization
    REQUIRE(map._h() == static_cast<int>(map_size/map_res));
    REQUIRE(map._w() == static_cast<int>(map_size/map_res));
    REQUIRE(map._d() == static_cast<int>(map_size/map_res));
    REQUIRE(map._map_res() == map_res);

    for (int x = 0; x < map._h(); ++x)
    {
        for (int y = 0; y < map._w(); ++y)
        {
            for (int z = 0; z < map._d(); ++z)
            {
                REQUIRE(map.at_index(x, y, z) == nullptr);
            }
        }
    }

    // Test insertion
    REQUIRE_FALSE(map.insert(map_size, 0, 0, 1));
    REQUIRE_FALSE(map.insert(0, map_size, 0, 1));
    REQUIRE_FALSE(map.insert(0, 0, map_size, 1));

    REQUIRE(map.insert(0.25, 0.25, 0.25, 1.0));
    REQUIRE(map.submap_at(0.25, 0.25, 0.25) == 1.0);
}

TEST_CASE("Map", "[map]")
{
    int map_size = 1;
    double map_res = 0.5;
    double subvoxel_res = 0.25;

    // Test both constructors
    {
        subvoxelmap::Parameters params(1, 0.5, 0.1);
        map::Map map(params);
        test_map(map, params.map_size, params.map_res, params.subvoxel_res);
    }
    {
        map::Map map(map_size, map_res, subvoxel_res);
        test_map(map, map_size, map_res, subvoxel_res);
    }
}

