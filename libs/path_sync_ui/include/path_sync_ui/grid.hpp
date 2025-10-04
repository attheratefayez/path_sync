/**
 * @file grid.hpp
 * @brief Header file of the Grid class for Path Sync Project.
 */

#ifndef __PATH_SYNC_GRID_HPP__
#define __PATH_SYNC_GRID_HPP__

#include <vector>
#include <SFML/Graphics.hpp>

#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_ui/cell.hpp"

namespace path_sync
{

class Grid : public sf::Drawable
{
public:
    Grid(const path_sync::MapData& map_data, int cell_size);
    void draw(sf::RenderTarget& target, sf::RenderStates states) const override;
    void sync_with_map_data(const path_sync::MapData& map_data);

private:
    int cell_size_;
    std::vector<std::vector<Cell>> drawable_grid_;
    void adjust_cell_size_(int height, int width);
};

} // namespace path_sync

#endif // !__PATH_SYNC_GRID_HPP__
