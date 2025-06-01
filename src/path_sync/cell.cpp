#include "path_sync/visualization_system/cell.hpp"
#include "path_sync/visualization_system/visualization_system_config.hpp"

namespace path_sync
{
void Cell::__set_color_for_type()
{
    switch (type)
    {
    case CellType::DEFAULT:
        setFillColor(sf::Color::White);
        break;

    case CellType::WALL:
        setFillColor(sf::Color(39, 55, 70));
        break;

    case CellType::FOUND:
        setFillColor(sf::Color::Green);
        break;

    case CellType::VISITED:
        setFillColor(sf::Color(39, 245, 71, 80));
        break;

    case CellType::START:
        setFillColor(sf::Color(44, 235, 6));
        break;

    case CellType::END:
        setFillColor(sf::Color::Red);
        break;

    case CellType::PATH:
        setFillColor(sf::Color::Blue);
        break;

    default:
        setFillColor(sf::Color::White);
    }
}

Cell::Cell(CellType cell_type, sf::Vector2f position) : type(cell_type)
{
    setSize(
        sf::Vector2f(path_sync::VisualizationSystemConfig::CELL_SIZE, path_sync::VisualizationSystemConfig::CELL_SIZE));
    setPosition(position);
    __set_color_for_type();
}

void Cell::set_cell_type(CellType cell_type)
{
    type = cell_type;
    __set_color_for_type();
}

CellType Cell::get_cell_type() const
{
    return type;
}

} // NAMESPACE path_sync
