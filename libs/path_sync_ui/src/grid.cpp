#include "path_sync_ui/grid.hpp"
#include "path_sync_ui/visualization_system_config.hpp"

namespace path_sync
{

Grid::Grid(const path_sync::MapData &map_data, int cell_size) : cell_size_(cell_size)
{
    adjust_cell_size_(map_data.get_height(), map_data.get_width());
    drawable_grid_.resize(map_data.get_height());
    for (int y = 0; y < map_data.get_height(); ++y)
    {
        drawable_grid_[y].reserve(map_data.get_width());
        for (int x = 0; x < map_data.get_width(); ++x)
        {
            drawable_grid_[y].emplace_back(
                map_data.get_cell_type(Coordinate(x, y)), x, y);
        }
    }
}

void Grid::sync_with_map_data(const path_sync::MapData &map_data)
{
    if (map_data.get_height() != drawable_grid_.size() ||
        map_data.get_width() != drawable_grid_[0].size())
        *this = std::move(Grid(map_data, 2));

    for (int y = 0; y < map_data.get_height(); ++y)
        for (int x = 0; x < map_data.get_width(); ++x)
            drawable_grid_[y][x].set_cell_type(
                map_data.get_cell_type(Coordinate(x, y)));
}

void Grid::adjust_cell_size_(int height, int width)
{
    int h = 950 / height;
    int w = 1850 / width;
    cell_size_ = (h > w) ? w : h;
    cell_size_ = (cell_size_ > 2) ? cell_size_ : 2;
}

} // namespace path_sync
