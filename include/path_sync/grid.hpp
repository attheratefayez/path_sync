/**
 * @file grid.hpp
 * @brief Header file of the Grid class for Path Sync Project.
 */

#ifndef __PATH_SYNC_GRID_HPP__
#define __PATH_SYNC_GRID_HPP__

#include <vector>
#include <SFML/Graphics.hpp>

#include "path_sync/cell.hpp"
#include "path_sync/psync_types.hpp"
#include "path_sync/visualization_system_config.hpp"


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
    std::vector<Coordinate> find_neighbors_b2(Coordinate candidate) const;
    void reset_grid();

    int get_num_of_rows() const {return __num_of_rows;}
    int get_num_of_cols() const {return __num_of_cols;}
    CellGrid& get_grid() {return __cell_grid;}
    Coordinate get_grid_size() { return {__num_of_rows, __num_of_cols}; }
    std::vector<Coordinate>& get_start_points() { return start_points; };
    std::vector<Coordinate>& get_end_points() { return end_points; };

private:
    int __num_of_rows;
    int __num_of_cols;
    CellGrid __cell_grid;
    std::vector<Coordinate> start_points;
    std::vector<Coordinate> end_points;
};

} // namespace psync

#endif // !__PATH_SYNC_GRID_HPP__
