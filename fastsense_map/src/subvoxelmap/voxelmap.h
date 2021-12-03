#pragma once

#include "util.h"
#include "subvoxelmap.h"
#include <fmt/printf.h>
#include <cassert>

namespace voxelmap
{

template <typename T>
class VoxelMap
{
public:
    VoxelMap(int map_size, double map_res, double subvoxel_res, T default_subvoxel_val)
    : map_size(map_size)
    , map_res(map_res)
    , h(static_cast<int>(map_size/map_res))
    , w(static_cast<int>(map_size/map_res))
    , d(static_cast<int>(map_size/map_res))
    , nelems(h * w * d)
    , map(new SubvoxelMap<T>*[h * w * d])
    , subvoxel_res(subvoxel_res)
    , offset(map_size/2.0)
    , default_subvoxel_val(default_subvoxel_val)
    {
        init_map();
    }

    ~VoxelMap()
    {
        for (int i = 0; i < h * w * d; ++i)
        {
            delete map[i];
        }

        delete[] map;
    }

    void clear()
    {
       throw std::runtime_error("NOT IMPLEMENTED");
    }

    VoxelMap& operator=(const VoxelMap<T>&) = delete;
    VoxelMap& operator=(VoxelMap<T>&&) = delete;
    VoxelMap(const voxelmap::VoxelMap<T>&) = delete;
    VoxelMap(VoxelMap<T>&&) = delete;

    const SubvoxelMap<T>* subvoxelmap_at_index(int x, int y, int z) const
    {
        if (not index_in_range(x, y, z))
        {
            throw std::runtime_error(fmt::format("Map accessed @ index ({}/{}/{}) with max index ({}/{]/{})", x, y, z, w, h, d).c_str());
        }

        int i = util::conv_3dindex_1dindex(x, y, z, w, d);
        return map[i];
    }

    template <typename F>
    const SubvoxelMap<T>* subvoxelmap_at_point(F& x, F& y, F& z) const
    {
        apply_offset(x, y, z);
        if (not index_in_range(x, y, z))
        {
            throw std::runtime_error(fmt::format("Map accessed with ({}/{}/{}) elements and map size {}", x, y, z, map_size).c_str());
        }

        int i = util::conv_3dpoint_1dindex(x, y, z, map_res, w, d);
        return map[i];
    }
    
    template <typename F>
    T value_at(F x, F y, F z) const
    {
        auto* submap = subvoxelmap_at_point(x, y, z);
        if (submap == nullptr)
        {
//            fmt::print("nullptr {} {}\n", default_subvoxel_val, default_subvoxel_val == NAN);
//            fmt::print("nullptr");
            return default_subvoxel_val;
        }
        util::Point<double> in_submap = to_submap_point(x, y, z);
        return submap->at_point(in_submap.x, in_submap.y, in_submap.z);
    }

    const SubvoxelMap<T>* at(int i) const
    {
        return subvoxelmap_at_index(i % w, (i / w) % h, i / (w * h));
    }

    template <typename F>
    bool insert(F x, F y, F z, T val)
    {
        apply_offset(x, y, z);
        if (not point_in_range(x, y, z))
        {
            return false;
        }

        if (submap_unititialized(x, y, z))
        {
            init_subvoxel_map(x, y, z);
        }

        insert_into_subvoxel_map(x, y, z, val);
        return true;
    }

    int _h() const
    {
        return h;
    }

    int _w() const
    {
        return w;
    }

    int _d() const
    {
        return d;
    }

    double _map_res() const
    {
        return map_res;
    }

private:
    int map_size;
    double map_res;
    int h;
    int w;
    int d;
    int nelems;
    SubvoxelMap<T>** map;
    double subvoxel_res;
    double offset;
    T default_subvoxel_val;

    void init_map()
    {
        std::fill_n(map, h * w * d, nullptr);
    }
    
    template <typename F>
    bool point_in_range(F x, F y, F z) const
    {
        return x >= 0 && x < map_size && y >= 0 && y < map_size && z >= 0 && z < map_size;
    }

    bool index_in_range(int x, int y, int z) const
    {
        return x >= 0 && x < w && y >= 0 && y < h && z >= 0 && z < d;
    }
    
    template <typename F>
    void apply_offset(F &x, F &y, F &z) const
    {
        x += offset;
        y += offset;
        z += offset;
    }
    
    template <typename F>
    void init_subvoxel_map(F x, F y, F z)
    {
        int subvoxel_elems_per_dim = static_cast<int>(map_res / subvoxel_res);
        map[util::conv_3dpoint_1dindex(x, y, z, map_res, w, d)] = new SubvoxelMap<T>(subvoxel_elems_per_dim, subvoxel_elems_per_dim, subvoxel_elems_per_dim, subvoxel_res, map_res, default_subvoxel_val);
    }

    template <typename F>
    void insert_into_subvoxel_map(F x, F y, F z, T val)
    {
        auto* subvoxelmap = map[util::conv_3dpoint_1dindex(x, y, z, map_res, w, d)];
        util::Point<double> submap_p = to_submap_point(x, y, z);
        subvoxelmap->insert(submap_p.x, submap_p.y, submap_p.z, val);
    }

    template <typename F>
    bool submap_unititialized(F x, F y, F z) const
    {
        int i = util::conv_3dpoint_1dindex(x, y, z, map_res, w, d);
        return map[i] == nullptr;
    }

    /**
     * Convert Point in 3d map coord. system into 3d point in submap coord. system
     *
     * "origin_idx" describes the origin in map index space and its respective idx of the submap that was just created
     *
     * To calculate the actual value that will be written into submap, we must convert "origin_idx" into map coord. space
     * -> origin_idx * map_res
     *
     * @param x x coord of point in 3d map coord system
     * @param y y coord of point in 3d map coord system
     * @param z z coord of point in 3d map coord system
     * @return point in submap coord. system
     */
    template <typename F>
    util::Point<double> to_submap_point(F x, F y, F z) const
    {
        util::Point<int> origin_idx = util::conv_3dpoint_3dindex(x, y, z, map_res);
        return {x - origin_idx.x * map_res, y - origin_idx.y * map_res, z - origin_idx.z * map_res};
    }
};

} // end namespace map