/**
 * @file grid.hpp
 * @brief Header file of the Grid class for Path Sync Project.
 */

#ifndef __PATH_SYNC_GRID_HPP__
#define __PATH_SYNC_GRID_HPP__

#include <vector>
#include <SFML/Graphics.hpp>

#include "path_sync/visualization_system/cell.hpp"
#include "path_sync/map_loader/env_map.hpp"
#include "path_sync/psync_types.hpp"
#include "path_sync/visualization_system/visualization_system_config.hpp"

enum GridMode
{
    Free,
    Map
};

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
    Grid(VisualizationSystemConfig& system_config, GridMode grid_mode = GridMode::Free);
    void draw(sf::RenderTarget& target, sf::RenderStates states) const override;
    std::vector<Coordinate> find_neighbors_b2(Coordinate candidate) const;
    void clear_paths();
    void reset_grid();
    void load_map(std::string map_name);
    void change_grid_mode();
    void change_current_map(std::string map_name);

    int get_num_of_rows() const {return __num_of_rows;}
    int get_num_of_cols() const {return __num_of_cols;}
    CellGrid& get_grid() {return __cell_grid;}
    Coordinate get_grid_size() { return {__num_of_rows, __num_of_cols}; }
    std::vector<Coordinate>& get_start_points() { return __start_points; };
    std::vector<Coordinate>& get_end_points() { return __end_points; };

private:
    VisualizationSystemConfig& __system_config;
    int __num_of_rows;
    int __num_of_cols;
    CellGrid __cell_grid;
    CostGrid __cost_grid;
    std::vector<Coordinate> __start_points;
    std::vector<Coordinate> __end_points;
    std::string __map_file_name;
    GridMode __current_grid_mode;
    Map __current_map;

    void __create_free_grid();
    void __create_map_grid();
    void __create_cost_grid();
    void __adjust_cell_size();
    void __imprint_map_in_cell_grid();
};

} // namespace psync

#endif // !__PATH_SYNC_GRID_HPP__
