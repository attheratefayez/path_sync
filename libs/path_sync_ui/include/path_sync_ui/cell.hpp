#ifndef __PATH_SYNC_CELL_HPP__
#define __PATH_SYNC_CELL_HPP__

#include <QColor>

#include "path_sync_core/path_sync_types.hpp"

namespace path_sync
{

struct Cell
{
    CellType type = CellType::DEFAULT;

    Cell() = default;
    Cell(CellType cell_type, int x, int y);

    void set_cell_type(CellType cell_type);
    CellType get_cell_type() const;

    int grid_x = 0;
    int grid_y = 0;
};

QColor cell_type_to_color(CellType type);

} // namespace path_sync
#endif
