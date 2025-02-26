#include <iostream>
#include "path_sync/cell.hpp"
#include "path_sync/logger.hpp"
#include "path_sync/grid.hpp"
#include "path_sync/visualization_system_config.hpp"

namespace psync {

Grid::Grid(VisualizationSystemConfig& system_config)
    : __num_of_rows(system_config.HEIGHT / psync::VisualizationSystemConfig::CELL_SIZE)
    , __num_of_cols(system_config.WIDTH / psync::VisualizationSystemConfig::CELL_SIZE)
{
    std::cout << "Calculated Grid: " << __num_of_rows << ", " << __num_of_cols << std::endl;

    for (int row_count = 0; row_count < __num_of_rows; ++row_count) {
        std::vector<psync::Cell> temp_row;
        for (int col_count = 0; col_count < __num_of_cols; ++col_count) {
            temp_row.emplace_back(
                Cell(psync::CellType::DEFAULT, 
                {
                    (float)(col_count * psync::VisualizationSystemConfig::CELL_SIZE),
                    (float)(row_count * psync::VisualizationSystemConfig::CELL_SIZE),
                
                })
            );
        }
        temp_row.shrink_to_fit();
        __cell_grid.push_back(temp_row);
    }

    std::cout << "Grid Created: " << __cell_grid.size() << ", " << __cell_grid[0].size() << std::endl;

}

void Grid::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    for(auto& row: __cell_grid)
    {
        for(const psync::Cell& cell: row)
        {
            target.draw(cell);
        }
    }
}


} // namespace psync
