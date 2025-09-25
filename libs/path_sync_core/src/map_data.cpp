#include "path_sync_core/map_loader/map_data.hpp"

namespace path_sync {

MapData::MapData(int width, int height)
    : width_(width), height_(height) {
    grid_data_.resize(height, std::vector<CellType>(width, CellType::DEFAULT));
}

CellType MapData::get_cell_type(int x, int y) const {
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        return grid_data_[y][x];
    }
    // Return a default or error value if out of bounds
    return CellType::WALL; 
}

void MapData::set_cell_type(int x, int y, CellType type) {
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        grid_data_[y][x] = type;
    }
}

} // namespace path_sync
