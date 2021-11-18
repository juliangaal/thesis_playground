#include "util.h"
#include "subvoxelmap.h"

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
                REQUIRE(std::isnan(map.at(x, y, z)));
            }
        }
    }

    // Test at limits (int)
    REQUIRE_THROWS(map.at(-1, 0, 0));
    REQUIRE_THROWS(map.at(0, -1, 0));
    REQUIRE_THROWS(map.at(0, 0, -1));
    REQUIRE_THROWS(map.at(map_size, 0, 0));
    REQUIRE_THROWS(map.at(0, map_size, 0));
    REQUIRE_THROWS(map.at(0, 0, map_size));

    // Test at limits (double)
    REQUIRE_THROWS(map.at(-1.0, 0.0, 0.0));
    REQUIRE_THROWS(map.at(0.0, -1.0, 0.0));
    REQUIRE_THROWS(map.at(0.0, 0.0, -1.0));
    REQUIRE_THROWS(map.at(static_cast<double>(map_size), 0.0, 0.0));
    REQUIRE_THROWS(map.at(0.0, static_cast<double>(map_size), 0.0));
    REQUIRE_THROWS(map.at(0.0, 0.0, static_cast<double>(map_size)));

    // Test insert and borders of inserted cube
    // insert at (0, 0, 0)
    map.insert(0, 0, 0, 3.0);
    REQUIRE(map.at(0, 0, 0) == 3.0);
    REQUIRE(map.at(0.0, 0.0, 0.0) == 3.0);
    REQUIRE(std::isnan(map.at(res, res, res)));
    REQUIRE(map.at(res-epsilon, res-epsilon, res-epsilon) == 3);

    REQUIRE_THROWS(map.insert(-1, 0, 0, 8));
    REQUIRE_THROWS(map.insert(0, -1, 0, 8));
    REQUIRE_THROWS(map.insert(0, 0, -1, 8));
    REQUIRE_THROWS(map.insert(map_size, 0, 0, 8));
    REQUIRE_THROWS(map.insert(0, map_size, 0, 8));
    REQUIRE_THROWS(map.insert(0, 0, map_size, 8));

    map.insert(1, 1, 1, 6.0);
    REQUIRE(map.at(1, 1, 1) == 6.0);
    REQUIRE(map.at(1.0, 1.0, 1.0) == 6.0);
    REQUIRE(std::isnan(map.at(1.0 + res, 1.0 + res, 1.0 + res)));
    REQUIRE(map.at(1.0 + res -epsilon, 1.0 + res-epsilon, 1.0 + res - epsilon) == 6.0);

    // Test clearing
    map.clear();
    for (int x = 0; x < map._h(); ++x)
    {
        for (int y = 0; y < map._w(); ++y)
        {
            for (int z = 0; z < map._d(); ++z)
            {
                REQUIRE(std::isnan(map.at(x, y, z)));
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

