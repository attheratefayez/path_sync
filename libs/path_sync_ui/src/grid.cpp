#include "path_sync_ui/grid.hpp"
#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_ui/visualization_system_config.hpp"

#include <iostream>
namespace path_sync
{

Grid::Grid(const path_sync::MapData &map_data, int cell_size)
    : cell_size_(cell_size)
{
    adjust_cell_size_(map_data.get_height(), map_data.get_width());

    drawable_grid_.resize(map_data.get_height());

    for (int y = 0; y < map_data.get_height(); ++y)
    {
        drawable_grid_[y].reserve(map_data.get_width());
        for (int x = 0; x < map_data.get_width(); ++x)
        {
            drawable_grid_[y].emplace_back(map_data.get_cell_type(Coordinate(x, y)),
                                           sf::Vector2f(x * cell_size_, y * cell_size_));

        }
    }
}

void Grid::draw(sf::RenderTarget &target, sf::RenderStates states) const
{
    for (const auto &row : drawable_grid_)
    {
        for (const auto &cell : row)
        {
            target.draw(cell, states);
        }
    }
}

void Grid::sync_with_map_data(const path_sync::MapData &map_data)
{
    for (int y = 0; y < map_data.get_height(); ++y)
    {
        for (int x = 0; x < map_data.get_width(); ++x)
        {
            path_sync::CellType temp_cell_type = map_data.get_cell_type(Coordinate(x, y));
            drawable_grid_[y][x].set_cell_type(temp_cell_type);
        }
    }
}

void Grid::adjust_cell_size_(int height, int width)
{
    int h, w;
    h = 950 / height;
    w = 1850 / width;

    cell_size_ = (h > w) ? w : h;
    cell_size_ = (cell_size_ > 2) ? cell_size_ : 2;
    cell_size_ = (int)cell_size_;

    path_sync::VisualizationSystemConfig::CELL_SIZE = cell_size_;
}

} // namespace path_sync
