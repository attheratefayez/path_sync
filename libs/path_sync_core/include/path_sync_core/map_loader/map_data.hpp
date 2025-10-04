#ifndef __PATH_SYNC_CORE_MAP_DATA_HPP__
#define __PATH_SYNC_CORE_MAP_DATA_HPP__

#include <vector>

#include "path_sync_core/path_sync_types.hpp"

namespace path_sync
{

/**
 * @class MapData
 * @details Stores Map data, but it is specifically designed to used in path_sync_core
 * and path_sync_ui. This class makes a grid like object of cells (CellType). So, the map is not a str
 * any more.
 */
class MapData
{
  public:
    MapData()
        : map_info_{}
    {
    }

    MapData(MapInfo const &map_info);

    int get_width() const
    {
        return map_info_.width;
    }
    int get_height() const
    {
        return map_info_.height;
    }

    MapInfo get_map_info() const
    {
        return map_info_;
    }

    CellType get_cell_type(Coordinate pos) const;
    void set_cell_type(Coordinate pos, CellType type);

  private:
    MapInfo map_info_;
    std::vector<std::vector<CellType>> grid_data_;

    void create_grid_data_();
};

} // namespace path_sync

#endif // __PATH_SYNC_CORE_MAP_DATA_HPP__
