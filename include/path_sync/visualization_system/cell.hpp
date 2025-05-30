/**
 * @file cell.hpp
 * @brief Header file for a single cell in the Drawing Grid / Map grid of the Path Sync Project.
 */

#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/System/Vector2.hpp>

#ifndef __PATH_SYNC_CELL_HPP__
#define __PATH_SYNC_CELL_HPP__

namespace path_sync {
enum CellType
{
    DEFAULT,
    WALL,
    FOUND,
    VISITED,
    START,
    END,
    PATH,
};

/**
 * @class Cell
 * @brief Class for a single cell in Grid (Drawing/Map).
 *
 */
class Cell : public sf::RectangleShape
{
private:
    /**
     * @brief Deleted default, copy and move constructors.
     */
    Cell() = delete;

    CellType type;
    void __set_color_for_type();

public:
        /**
     * @brief Creates a Cell with given type and position. 
     *
     * @param cell_type a type of cell in path_sync::CellType.
     * @param position position of the cell to be placed.
     */
    Cell(CellType cell_type, sf::Vector2f position);

    /**
     * @brief Sets the type of a cell. 
     *
     * @param cell_type a type of cell in path_sync::CellType.
     */
    void set_cell_type(CellType cell_type);

    /**
     * @brief returns the type of cell.
     *
     * @return path_sync::CellType
     */
    CellType get_cell_type() const;
};
} // end of namespace path_sync 
#endif // !__PATH_SYNC_CELL_HPP__
