#include <iostream>
#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/logger.hpp"
#include "path_sync_core/path_sync_types.hpp"

namespace path_sync
{

MapData::MapData(MapInfo const &map_info)
    : map_info_{map_info}
{

    grid_data_.resize(map_info_.height, std::vector<CellType>(map_info_.width, CellType::DEFAULT));
    create_grid_data_();
}

CellType MapData::get_cell_type(Coordinate pos) const
{
    int x, y;
    x = pos.first;
    y = pos.second;

    if (x >= 0 && x < map_info_.width && y >= 0 && y < map_info_.height)
    {
        return grid_data_[y][x];
    }
    // Return a default or error value if out of bounds
    return CellType::WALL;
}

void MapData::set_cell_type(Coordinate pos, CellType type)
{
    int x, y;
    x = pos.first;
    y = pos.second;

    if (x >= 0 && x < map_info_.width && y >= 0 && y < map_info_.height)
    {
        grid_data_[y][x] = type;
    }
}

void MapData::create_grid_data_()
{
    std::string line;
    int row = 0, col = 0;

    while(std::getline(map_info_.map, line))
    {
        col = 0;
        for(auto elem: line)
        {
            CellType type = CellType::WALL;

            if(elem == '.')
                type = CellType::DEFAULT;

            grid_data_[row][col] = type;
            ++col;
        }
        row++;

    }

}

} // namespace path_sync
