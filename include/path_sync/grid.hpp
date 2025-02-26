/**
 * @file grid.hpp
 * @brief Header file of the Grid class for Path Sync Project.
 */

#ifndef __PATH_SYNC_GRID_HPP__
#define __PATH_SYNC_GRID_HPP__

#include <vector>
#include <SFML/Graphics.hpp>

#include "path_sync/cell.hpp"
#include "path_sync/visualization_system_config.hpp"

typedef std::vector<std::vector<psync::Cell>> CellGrid;

namespace psync
{

/**
 * @class Grid
 * @brief Grid class that creates drawing grid or map grid with Cell.
 *
 */
class Grid : public sf::Drawable
{
public: 
    Grid(VisualizationSystemConfig& system_config);
    void draw(sf::RenderTarget& target, sf::RenderStates states) const override;

    int get_num_of_rows() const {return __num_of_rows;}
    int get_num_of_cols() const {return __num_of_cols;}
    CellGrid& get_grid() {return __cell_grid;}
    bool grid_changed() const {return __grid_changed;}
    void set_grid_changed(bool state) {__grid_changed = state;}

private:
    int __num_of_rows;
    int __num_of_cols;
    CellGrid __cell_grid;
    mutable bool __grid_changed;
    /*sf::RenderWindow& __render_window_ref;*/
};

} // namespace psync

#endif // !__PATH_SYNC_GRID_HPP__
