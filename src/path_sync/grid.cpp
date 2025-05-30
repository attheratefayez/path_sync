#include <cstddef>
#include <iostream>
#include <sstream>

#include "path_sync/logging/logger.hpp"
#include "path_sync/visualization_system/cell.hpp"
#include "path_sync/visualization_system/grid.hpp"
#include "path_sync/visualization_system/visualization_system_config.hpp"

namespace path_sync
{

Grid::Grid(VisualizationSystemConfig &system_config, GridMode grid_mode)
    : __num_of_rows(system_config.HEIGHT / path_sync::VisualizationSystemConfig::CELL_SIZE),
      __num_of_cols(system_config.WIDTH / path_sync::VisualizationSystemConfig::CELL_SIZE), __current_grid_mode(grid_mode),
      __map_file_name("AR0015SR"), __system_config(system_config)
{
    std::cout << "Calculated Grid: " << __num_of_rows << ", " << __num_of_cols << std::endl;

    if (__current_grid_mode == GridMode::Free)
        __create_free_grid();
    else
        __create_map_grid();

    std::cout << "Grid Created: " << __cell_grid.size() << ", " << __cell_grid[0].size() << std::endl;
    std::cout << "Cost grid size: " << __cost_grid.size() << " " << __cost_grid[0].size() << " "
              << __cost_grid[0][0].size() << std::endl;
}

void Grid::draw(sf::RenderTarget &target, sf::RenderStates states) const
{
    for (auto &row : __cell_grid)
    {
        for (const path_sync::Cell &cell : row)
        {
            target.draw(cell);
        }
    }
}

std::vector<Coordinate> Grid::find_neighbors_b2(Coordinate candidate) const
{
    const int x_moves[] = {0, 1, 0, -1};
    const int y_moves[] = {1, 0, -1, 0};

    std::vector<Coordinate> neighbours;

    for (int i = 0; i < 4; i++)
    {
        int n_col = candidate.first + y_moves[i];
        int n_row = candidate.second + x_moves[i];

        if ((n_row < 0 or n_row >= __num_of_rows))
        {
            /*std::cout << "Cont Row: " << n_row << ", " << n_col << " limit: " <<
             * __num_of_rows << std::endl;*/
            continue;
        }
        if ((n_col < 0 or n_col >= __num_of_cols))
        {
            /*std::cout << "Cont Col: " << n_row << ", " << n_col << " limit: " <<
             * __num_of_cols << std::endl;*/
            continue;
        }

        if (__cell_grid[n_row][n_col].get_cell_type() == CellType::WALL)
            continue;

        neighbours.push_back(Coordinate(n_col, n_row));
    }

    return neighbours;
}

void Grid::clear_paths()
{
    for (std::vector<Cell> &row : __cell_grid)
    {
        for (path_sync::Cell &cell : row)
        {
            if (cell.get_cell_type() == path_sync::CellType::PATH)
                cell.set_cell_type(path_sync::CellType::DEFAULT);
        }
    }
}

void Grid::reset_grid()
{
    for (std::vector<Cell> &row : __cell_grid)
    {
        for (path_sync::Cell &cell : row)
        {
            cell.set_cell_type(path_sync::CellType::DEFAULT);
        }
    }

    __start_points.clear();
    __end_points.clear();
}

void Grid::load_map(std::string map_name)
{
    Map temp_map(map_name);
    __current_map = temp_map;
}

void Grid::change_grid_mode()
{
    reset_grid();

    if (__current_grid_mode == GridMode::Free)
    {
        __create_map_grid();
        __current_grid_mode = GridMode::Map;
    }

    else
    {
        __create_free_grid();
        __current_grid_mode = GridMode::Free;
    }
}

void Grid::change_current_map(std::string map_name)
{
    reset_grid();
    __cell_grid.clear();
    __cost_grid.clear();

    __map_file_name = map_name;
    __create_map_grid();
}

void Grid::__create_free_grid()
{
    for (int row_count = 0; row_count < __num_of_rows; ++row_count)
    {
        std::vector<path_sync::Cell> temp_row;
        for (int col_count = 0; col_count < __num_of_cols; ++col_count)
        {
            temp_row.emplace_back(
                Cell(path_sync::CellType::DEFAULT, {
                                                   (float)(col_count * path_sync::VisualizationSystemConfig::CELL_SIZE),
                                                   (float)(row_count * path_sync::VisualizationSystemConfig::CELL_SIZE),

                                               }));
        }
        temp_row.shrink_to_fit();
        __cell_grid.push_back(temp_row);
    }

    __create_cost_grid();
}

void Grid::__create_map_grid()
{
    load_map(__map_file_name);
    __num_of_rows = __current_map.get_map_height();
    __num_of_cols = __current_map.get_map_width();
    __adjust_cell_size();
    __create_free_grid();
    __imprint_map_in_cell_grid();
    __create_cost_grid();
}

void Grid::__create_cost_grid()
{
    if (path_sync::VisualizationSystemConfig::NUM_OF_OBJECTIVES == 1)
    {
        for (int r = 0; r < __num_of_rows; ++r)
        {
            std::vector<std::vector<size_t>> temp_row;
            for (int c = 0; c < __num_of_cols; ++c)
            {
                temp_row.push_back(std::vector<size_t>{1});
            }
            temp_row.shrink_to_fit();
            __cost_grid.push_back(temp_row);
        }
    }

    __cost_grid.resize(__num_of_rows);
    __cost_grid[0].resize(__num_of_cols);
    __cost_grid[0][0].resize(path_sync::VisualizationSystemConfig::NUM_OF_OBJECTIVES);
}

void Grid::__adjust_cell_size()
{
    int temp_width = (int)(__system_config.WIDTH / __num_of_cols);
    int temp_height = (int)(__system_config.HEIGHT / __num_of_rows);

    temp_width = temp_width > 0 ? temp_width : 1;
    temp_height = temp_height > 0 ? temp_height : 1;

    __system_config.CELL_SIZE = temp_width < temp_height ? temp_width : temp_height;

    std::stringstream ss;
    ss << "Cell Size Adjusted: " << __system_config.CELL_SIZE << std::endl;

    path_sync::Logger::get()->info(ss.str().c_str());
}

void Grid::__imprint_map_in_cell_grid()
{
    std::string line;
    size_t row_counter = 0;

    /*std::cout << "fn " << __current_map.get_map_name() << " " << __current_map.get_map_width() << " " <<
     * __current_map.get_map_height() << std::endl;*/

    while (std::getline(__current_map.get_map(), line))
    {
        for (size_t col_counter = 0; col_counter < __num_of_cols; ++col_counter)
        {
            path_sync::CellType type = path_sync::CellType::DEFAULT;
            if (line[col_counter] == '@' or line[col_counter] == 'O' or line[col_counter] == 'T' or
                line[col_counter] == 'W')
            {
                type = path_sync::CellType::WALL;
            }

            __cell_grid[row_counter][col_counter].set_cell_type(type);

            /*std::cout << "imprinting: " << row_counter << ", " << col_counter << std::endl;*/
        }
        ++row_counter;
    }

    __current_map.get_map().clear();
    __current_map.get_map().seekg(0);
}

} // namespace path_sync
