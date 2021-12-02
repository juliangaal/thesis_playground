/**
 * @author Steffen Hinderink
 * @author Juri Vana
 */

#include "local_map.h"
#include "subvoxelmap/voxelmap.h"
#include <catch2/catch_test_macros.hpp>
#include <fmt/printf.h>

using namespace fastsense::map;
using Eigen::Vector3i;

constexpr int DEFAULT_VALUE = 4;
constexpr int DEFAULT_WEIGHT = 6;

TEST_CASE("Subvoxelmap_double", "[subvoxelmap_double]")
{
    double epsilon = 0.0001;
    double res = 1.0;
    int map_size = 4;
    map::SubvoxelMap<double> map(4, 1, NAN);
    
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
}

TEST_CASE("Subvoxelmap_TSDFEntry", "[subvoxelmap_tsdfentry]")
{
    double epsilon = 0.0001;
    double res = 1.0;
    int map_size = 4;
    TSDFEntry default_tsdf(DEFAULT_VALUE, DEFAULT_WEIGHT);
    map::SubvoxelMap<TSDFEntry> map(4, 1, default_tsdf);
    
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
                REQUIRE(map.at_index(x, y, z) == default_tsdf);
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
    map.insert(0, 0, 0, TSDFEntry(5, 5));
    REQUIRE(map.at_index(0, 0, 0) == TSDFEntry(5, 5));
    REQUIRE(map.at_point(0.0, 0.0, 0.0) == TSDFEntry(5, 5));
    REQUIRE(map.at_point(res, res, res) == default_tsdf);
    REQUIRE(map.at_point(res-epsilon, res-epsilon, res-epsilon) == TSDFEntry(5, 5));
    
    REQUIRE_THROWS(map.insert(-1, 0, 0, TSDFEntry(6, 6)));
    REQUIRE_THROWS(map.insert(0, -1, 0, TSDFEntry(6, 6)));
    REQUIRE_THROWS(map.insert(0, 0, -1, TSDFEntry(6, 6)));
    REQUIRE_THROWS(map.insert(map_size, 0, 0, TSDFEntry(6, 6)));
    REQUIRE_THROWS(map.insert(0, map_size, 0, TSDFEntry(6, 6)));
    REQUIRE_THROWS(map.insert(0, 0, map_size, TSDFEntry(6, 6)));

    map.insert(1, 1, 1, TSDFEntry(6, 6));
    REQUIRE(map.at_index(1, 1, 1) == TSDFEntry(6, 6));
    REQUIRE(map.at_point(1.0, 1.0, 1.0) == TSDFEntry(6, 6));
    REQUIRE(map.at_point(1.0 + res, 1.0 + res, 1.0 + res) == default_tsdf);
    REQUIRE(map.at_point(1.0 + res -epsilon, 1.0 + res-epsilon, 1.0 + res - epsilon) == TSDFEntry(6, 6));

//     Test clearing
    map.clear();
    for (int x = 0; x < map._h(); ++x)
    {
        for (int y = 0; y < map._w(); ++y)
        {
            for (int z = 0; z < map._d(); ++z)
            {
                REQUIRE(map.at_index(x, y, z) == default_tsdf);
            }
        }
    }
}

template <typename T>
bool n_initialized_submaps(map::VoxelMap<T>& map, int n)
{
    int counter = 0;
    for (int x = 0; x < map._h(); ++x)
    {
        for (int y = 0; y < map._w(); ++y)
        {
            for (int z = 0; z < map._d(); ++z)
            {
                if (map.subvoxelmap_at_index(x, y, z) != nullptr)
                {
                    ++counter;
                }
            }
        }
    }
    
    return counter == n;
}

template <typename T>
void test_map(map::VoxelMap<T>& map, int map_size, double map_res, double subvoxel_res, T insertion_value)
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
                REQUIRE(map.subvoxelmap_at_index(x, y, z) == nullptr);
            }
        }
    }
    
    // Test insertion out of range
    REQUIRE_FALSE(map.insert(map_size, 0, 0, insertion_value));
    REQUIRE_FALSE(map.insert(0, map_size, 0, insertion_value));
    REQUIRE_FALSE(map.insert(0, 0, map_size, insertion_value));
    REQUIRE_FALSE(map.insert(-map_size, 0, 0, insertion_value));
    REQUIRE_FALSE(map.insert(0, -map_size, 0, insertion_value));
    REQUIRE_FALSE(map.insert(0, 0, -map_size, insertion_value));
    
    int submaps_intialized = 0;

    REQUIRE(map.insert(0.25, 0.25, 0.25, insertion_value));
    REQUIRE(map.value_at(0.25, 0.25, 0.25) == insertion_value);
    REQUIRE(n_initialized_submaps(map, ++submaps_intialized));

    REQUIRE(map.insert(0.25, 0.75, 0.75, insertion_value));
    REQUIRE(map.value_at(0.25, 0.75, 0.75) == insertion_value);
    REQUIRE(n_initialized_submaps(map, ++submaps_intialized));

    REQUIRE(map.insert(0.75, 0.25, 0.75, insertion_value));
    REQUIRE(map.value_at(0.75, 0.25, 0.75) == insertion_value);
    REQUIRE(n_initialized_submaps(map, ++submaps_intialized));

    REQUIRE(map.insert(0.75, 0.75, 0.25, insertion_value));
    REQUIRE(map.value_at(0.75, 0.75, 0.25) == insertion_value);
    REQUIRE(n_initialized_submaps(map, ++submaps_intialized));

    REQUIRE(map.insert(0.75, 0.75, 0.75, insertion_value));
    REQUIRE(map.value_at(0.75, 0.75, 0.75) == insertion_value);
    REQUIRE(n_initialized_submaps(map, ++submaps_intialized));

    REQUIRE(map.insert(0.75, 0.25, 0.25, insertion_value));
    REQUIRE(map.value_at(0.75, 0.25, 0.25) == insertion_value);
    REQUIRE(n_initialized_submaps(map, ++submaps_intialized));

    REQUIRE(map.insert(0.25, 0.75, 0.25, insertion_value));
    REQUIRE(map.value_at(0.25, 0.75, 0.25) == insertion_value);
    REQUIRE(n_initialized_submaps(map, ++submaps_intialized));

    REQUIRE(map.insert(0.25, 0.25, 0.75, insertion_value));
    REQUIRE(map.value_at(0.25, 0.25, 0.75) == insertion_value);
    REQUIRE(n_initialized_submaps(map, ++submaps_intialized));

    REQUIRE(map.insert(-0.25, -0.25, -0.25, insertion_value));
    REQUIRE(map.value_at(-0.25, -0.25, -0.25) == insertion_value);
    REQUIRE(n_initialized_submaps(map, ++submaps_intialized));

    REQUIRE(map.insert(-0.25,- 0.75, -0.75, insertion_value));
    REQUIRE(map.value_at(-0.25, -0.75, -0.75) == insertion_value);
    REQUIRE(n_initialized_submaps(map, ++submaps_intialized));

    REQUIRE(map.insert(-0.75, -0.25, -0.75, insertion_value));
    REQUIRE(map.value_at(-0.75, -0.25, -0.75) == insertion_value);
    REQUIRE(n_initialized_submaps(map, ++submaps_intialized));

    REQUIRE(map.insert(-0.75, -0.75, -0.25, insertion_value));
    REQUIRE(map.value_at(-0.75, -0.75, -0.25) == insertion_value);
    REQUIRE(n_initialized_submaps(map, ++submaps_intialized));

    REQUIRE(map.insert(-0.75, -0.75, -0.75, insertion_value));
    REQUIRE(map.value_at(-0.75, -0.75, -0.75) == insertion_value);
    REQUIRE(n_initialized_submaps(map, ++submaps_intialized));

    REQUIRE(map.insert(-0.75, -0.25, -0.25, insertion_value));
    REQUIRE(map.value_at(-0.75, -0.25, -0.25) == insertion_value);
    REQUIRE(n_initialized_submaps(map, ++submaps_intialized));

    REQUIRE(map.insert(-0.25, -0.75, -0.25, insertion_value));
    REQUIRE(map.value_at(-0.25, -0.75, -0.25) == insertion_value);
    REQUIRE(n_initialized_submaps(map, ++submaps_intialized));

    REQUIRE(map.insert(-0.25, -0.25, -0.75, insertion_value));
    REQUIRE(map.value_at(-0.25, -0.25, -0.75) == insertion_value);
    REQUIRE(n_initialized_submaps(map, ++submaps_intialized));
}

TEST_CASE("Map", "[map]")
{
    int map_size = 2;
    double map_res = 0.5;
    double subvoxel_res = 0.25;
    
    {
        double default_value = NAN;
        double insertion_value = 1.0;
        map::VoxelMap<double> map(map_size, map_res, subvoxel_res, default_value);
        REQUIRE(std::isnan(map.value_at(0.25, 0.25, 0.25)));
        REQUIRE(std::isnan(map.value_at(0.25, 0.75, 0.75)));
        REQUIRE(std::isnan(map.value_at(0.75, 0.25, 0.75)));
        REQUIRE(std::isnan(map.value_at(0.75, 0.75, 0.25)));
        REQUIRE(std::isnan(map.value_at(0.75, 0.75, 0.75)));
        REQUIRE(std::isnan(map.value_at(0.75, 0.25, 0.25)));
        REQUIRE(std::isnan(map.value_at(0.25, 0.75, 0.25)));
        REQUIRE(std::isnan(map.value_at(0.25, 0.25, 0.75)));
        REQUIRE(std::isnan(map.value_at(-0.25, -0.25, -0.25)));
        REQUIRE(std::isnan(map.value_at(-0.25, -0.75, -0.75)));
        REQUIRE(std::isnan(map.value_at(-0.75, -0.25, -0.75)));
        REQUIRE(std::isnan(map.value_at(-0.75, -0.75, -0.25)));
        REQUIRE(std::isnan(map.value_at(-0.75, -0.75, -0.75)));
        REQUIRE(std::isnan(map.value_at(-0.75, -0.25, -0.25)));
        REQUIRE(std::isnan(map.value_at(-0.25, -0.75, -0.25)));
        REQUIRE(std::isnan(map.value_at(-0.25, -0.25, -0.75)));
        test_map(map, map_size, map_res, subvoxel_res, insertion_value);
    }
    
    {
        TSDFEntry default_value(DEFAULT_VALUE, DEFAULT_WEIGHT);
        TSDFEntry insertion_value(DEFAULT_VALUE+2, DEFAULT_WEIGHT+2);
        map::VoxelMap<TSDFEntry> map(map_size, map_res, subvoxel_res, default_value);
        REQUIRE(map.value_at(0.25, 0.25, 0.25) == default_value);
        REQUIRE(map.value_at(0.25, 0.75, 0.75) == default_value);
        REQUIRE(map.value_at(0.75, 0.25, 0.75) == default_value);
        REQUIRE(map.value_at(0.75, 0.75, 0.25) == default_value);
        REQUIRE(map.value_at(0.75, 0.75, 0.75) == default_value);
        REQUIRE(map.value_at(0.75, 0.25, 0.25) == default_value);
        REQUIRE(map.value_at(0.25, 0.75, 0.25) == default_value);
        REQUIRE(map.value_at(0.25, 0.25, 0.75) == default_value);
        REQUIRE(map.value_at(-0.25, -0.25, -0.25) == default_value);
        REQUIRE(map.value_at(-0.25, -0.75, -0.75) == default_value);
        REQUIRE(map.value_at(-0.75, -0.25, -0.75) == default_value);
        REQUIRE(map.value_at(-0.75, -0.75, -0.25) == default_value);
        REQUIRE(map.value_at(-0.75, -0.75, -0.75) == default_value);
        REQUIRE(map.value_at(-0.75, -0.25, -0.25) == default_value);
        REQUIRE(map.value_at(-0.25, -0.75, -0.25) == default_value);
        REQUIRE(map.value_at(-0.25, -0.25, -0.75) == default_value);
        test_map(map, map_size, map_res, subvoxel_res, insertion_value);
    }
}

TEST_CASE("FastSenseMap", "[FastSenseMap]")
{
    std::shared_ptr<GlobalMap> gm_ptr = std::make_shared<GlobalMap>("MapTest.h5", DEFAULT_VALUE, DEFAULT_WEIGHT);
    LocalMap localMap{5, 5, 5, gm_ptr};
    
    /*
     * Write some tsdf values and weights into the local map,
     * that will be written to the file as one chunk (-1_0_0)
     *
     *    y
     *    ^
     *  4 | .   .   .   . | .   .
     *    | Chunk -1_0_0  | Chunk 0_0_0
     *  3 | .   .   .   . | .   .
     *    |               |
     *  2 | .   .  p0  p1 | .   .
     *    |               |
     *  1 | .   .  p2  p3 | .   .
     *    |               |
     *  0 | .   .  p4  p5 | .   .
     *    | --------------+------
     * -1 | .   .   .   . | .   .
     *    | Chunk -1_-1_0 | Chunk 0_-1_0
     * -2 | .   .   .   . | .   .
     *    +-----------------------> x
     *   / -4  -3  -2  -1   0   1
     * z=0
     */
    TSDFEntry p0(0, 0);
    TSDFEntry p1(1, 1);
    TSDFEntry p2(2, 1);
    TSDFEntry p3(3, 2);
    TSDFEntry p4(4, 3);
    TSDFEntry p5(5, 5);
    localMap.value(-2, 2, 0) = p0;
    localMap.value(-1, 2, 0) = p1;
    localMap.value(-2, 1, 0) = p2;
    localMap.value(-1, 1, 0) = p3;
    localMap.value(-2, 0, 0) = p4;
    localMap.value(-1, 0, 0) = p5;
    
    // test getter
    CHECK(localMap.get_pos() == Vector3i(0, 0, 0));
    CHECK(localMap.get_size() == Vector3i(5, 5, 5));
    CHECK(localMap.get_offset() == Vector3i(2, 2, 2));
    
    // test in_bounds
    CHECK(localMap.in_bounds(0, 2, -2));
    CHECK(!localMap.in_bounds(22, 0, 0));
    // test default values
    CHECK(localMap.value(0, 0, 0).value() == DEFAULT_VALUE);
    CHECK(localMap.value(0, 0, 0).weight() == DEFAULT_WEIGHT);
    // test value access
    CHECK(localMap.value(-1, 2, 0).value() == 1);
    CHECK(localMap.value(-1, 2, 0).weight() == 1);
    
    // ==================== shift ====================
    // shift so that the chunk gets unloaded
    // Each shift can only cover an area of one Map size
//    localMap.shift(Vector3i( 5, 0, 0));
//    localMap.shift(Vector3i(10, 0, 0));
//    localMap.shift(Vector3i(15, 0, 0));
//    localMap.shift(Vector3i(20, 0, 0));
//    localMap.shift(Vector3i(24, 0, 0));
//
//    CHECK(localMap.get_pos() == Vector3i(24, 0, 0));
//    CHECK(localMap.get_size() == Vector3i(5, 5, 5));
//    CHECK(localMap.get_offset() == Vector3i(26 % 5, 2, 2));
//
//     test in_bounds
//    CHECK(!localMap.in_bounds(0, 2, -2));
//    CHECK(localMap.in_bounds(22, 0, 0));
//     test values
//    CHECK(localMap.value(24, 0, 0).value() == DEFAULT_VALUE);
//    CHECK(localMap.value(24, 0, 0).weight() == DEFAULT_WEIGHT);
//
//     ==================== shift directions ====================
//    localMap.value(24, 0, 0) = TSDFEntry(24, 0);
//
//    localMap.shift(Vector3i(24, 5, 0));
//    localMap.value(24, 5, 0) = TSDFEntry(24, 5);
//
//    localMap.shift(Vector3i(19, 5, 0));
//    localMap.value(19, 5, 0) = TSDFEntry(19, 5);
//
//    localMap.shift(Vector3i(19, 0, 0));
//    localMap.value(19, 0, 0) = TSDFEntry(19, 0);
//
//    localMap.shift(Vector3i(24, 0, 0));
//    CHECK(localMap.value(24, 0, 0).value() == 24);
//    CHECK(localMap.value(24, 0, 0).weight() == 0);
//
//    localMap.shift(Vector3i(19, 0, 0));
//    CHECK(localMap.value(19, 0, 0).value() == 19);
//    CHECK(localMap.value(19, 0, 0).weight() == 0);
//    localMap.shift(Vector3i(24, 5, 0));
//    CHECK(localMap.value(24, 5, 0).value() == 24);
//    CHECK(localMap.value(24, 5, 0).weight() == 5);
//    localMap.shift(Vector3i(19, 5, 0));
//    CHECK(localMap.value(19, 5, 0).value() == 19);
//    CHECK(localMap.value(19, 5, 0).weight() == 5);
//    localMap.shift(Vector3i(24, 0, 0));
//    CHECK(localMap.value(24, 0, 0).value() == 24);
//    CHECK(localMap.value(24, 0, 0).weight() == 0);
//
//     ==================== shift back ====================
//    localMap.shift(Vector3i(19, 0, 0));
//    localMap.shift(Vector3i(14, 0, 0));
//    localMap.shift(Vector3i( 9, 0, 0));
//    localMap.shift(Vector3i( 4, 0, 0));
//    localMap.shift(Vector3i( 0, 0, 0));
//
//     test return correct
//    CHECK(localMap.get_pos() == Vector3i(0, 0, 0));
//    CHECK(localMap.get_size() == Vector3i(5, 5, 5));
//    CHECK(localMap.get_offset() == Vector3i(2, 2, 2));
//
//     test in_bounds
//    CHECK(localMap.in_bounds(0, 2, -2));
//    CHECK(!localMap.in_bounds(22, 0, 0));
//
//    CHECK(localMap.value(0, 0, 0).value() == DEFAULT_VALUE);
//    CHECK(localMap.value(0, 0, 0).weight() == DEFAULT_WEIGHT);
//    CHECK(localMap.value(-1, 2, 0).value() == 1);
//    CHECK(localMap.value(-1, 2, 0).weight() == 1);
//
    // ==================== kernel ====================
    
    // Manipulate the map by applying a kernel that doubles the tsdf values and halfes the weights and shifting
//    auto q = FPGAManager::create_command_queue();
//    LocalMapTestKernel krnl{q};
//    krnl.run(localMap);
//    krnl.waitComplete();
//
//     Test manipulated map
//    CHECK(localMap.value(0, 0, 0).value() == DEFAULT_VALUE * 2);
//    CHECK(localMap.value(0, 0, 0).weight() == DEFAULT_WEIGHT / 2);
//    CHECK(localMap.value(-1, 2, 0).value() == 2);
//    CHECK(localMap.value(-1, 2, 0).weight() == 0);
    
    // Test persistent storage in HDF5 file
//    localMap.write_back();
    
//    HighFive::File f("MapTest.h5", HighFive::File::OpenOrCreate);
//    HighFive::Group g = f.getGroup("/map");
//    HighFive::DataSet d = g.getDataSet("-1_0_0");
//    std::vector<TSDFEntry::RawType> chunk;
//    d.read(chunk);
//
//    constexpr int CHUNK_SIZE = GlobalMap::CHUNK_SIZE;
//
//    CHECK(TSDFEntry(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 2) + CHUNK_SIZE * 2)]).value() == 0 * 2);
//    CHECK(TSDFEntry(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 1) + CHUNK_SIZE * 2)]).value() == 1 * 2);
//    CHECK(TSDFEntry(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 2) + CHUNK_SIZE * 1)]).value() == 2 * 2);
//    CHECK(TSDFEntry(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 1) + CHUNK_SIZE * 1)]).value() == 3 * 2);
//    CHECK(TSDFEntry(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 2) + CHUNK_SIZE * 0)]).value() == 4 * 2);
//    CHECK(TSDFEntry(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 1) + CHUNK_SIZE * 0)]).value() == 5 * 2);
//
//    CHECK(TSDFEntry(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 2) + CHUNK_SIZE * 2)]).weight() == 0 / 2);
//    CHECK(TSDFEntry(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 1) + CHUNK_SIZE * 2)]).weight() == 1 / 2);
//    CHECK(TSDFEntry(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 2) + CHUNK_SIZE * 1)]).weight() == 1 / 2);
//    CHECK(TSDFEntry(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 1) + CHUNK_SIZE * 1)]).weight() == 2 / 2);
//    CHECK(TSDFEntry(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 2) + CHUNK_SIZE * 0)]).weight() == 3 / 2);
//    CHECK(TSDFEntry(chunk[(CHUNK_SIZE * CHUNK_SIZE * (CHUNK_SIZE - 1) + CHUNK_SIZE * 0)]).weight() == 5 / 2);
}
