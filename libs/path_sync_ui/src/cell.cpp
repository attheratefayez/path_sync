#include "path_sync_ui/cell.hpp"

namespace path_sync
{

Cell::Cell(CellType cell_type, int x, int y)
    : type(cell_type)
    , grid_x(x)
    , grid_y(y)
{
}

void Cell::set_cell_type(CellType cell_type)
{
    type = cell_type;
}

CellType Cell::get_cell_type() const
{
    return type;
}

QColor cell_type_to_color(CellType type)
{
    switch (type)
    {
    case CellType::DEFAULT: return QColor(255, 255, 255);
    case CellType::WALL:    return QColor(39, 55, 70);
    case CellType::FOUND:   return QColor(0, 255, 0);
    case CellType::VISITED: return QColor(39, 245, 71, 80);
    case CellType::START:   return QColor(44, 235, 6);
    case CellType::END:     return QColor(255, 0, 0);
    case CellType::PATH:    return QColor(0, 0, 255);
    default:                return QColor(255, 255, 255);
    }
}

} // namespace path_sync
