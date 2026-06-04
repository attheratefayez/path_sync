#ifndef __PATH_SYNC_GRID_HPP__
#define __PATH_SYNC_GRID_HPP__

#include <vector>

#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_ui/cell.hpp"

namespace path_sync
{

class Grid
{
public:
    Grid() = default;
    Grid(const path_sync::MapData& map_data, int cell_size);

    void sync_with_map_data(const path_sync::MapData& map_data);

    int get_cell_size() const { return cell_size_; }
    int get_width() const { return static_cast<int>(drawable_grid_.empty() ? 0 : drawable_grid_[0].size()); }
    int get_height() const { return static_cast<int>(drawable_grid_.size()); }
    const Cell& get_cell(int x, int y) const { return drawable_grid_[y][x]; }

private:
    int cell_size_ = 5;
    std::vector<std::vector<Cell>> drawable_grid_;
    void adjust_cell_size_(int height, int width);
};

} // namespace path_sync
#endif
