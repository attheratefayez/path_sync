#ifndef __PATH_SYNC_CORE_MAP_DATA_HPP__
#define __PATH_SYNC_CORE_MAP_DATA_HPP__

#include <vector>

#include "path_sync_core/path_sync_types.hpp"

namespace path_sync {

class MapData {
public:
    MapData(int width, int height);

    int get_width() const { return width_; }
    int get_height() const { return height_; }
    CellType get_cell_type(int x, int y) const;
    void set_cell_type(int x, int y, CellType type);

private:
    int width_;
    int height_;
    std::vector<std::vector<CellType>> grid_data_;
};

} // namespace path_sync

#endif // __PATH_SYNC_CORE_MAP_DATA_HPP__
