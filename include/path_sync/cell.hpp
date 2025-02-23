#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/System/Vector2.hpp>

#ifndef __PATH_SYNC_CELL_HPP__
#define __PATH_SYNC_CELL_HPP__

namespace psync {
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

class Cell : public sf::RectangleShape
{
private:
    CellType type;

    void __set_color_for_type();

public:
    Cell()
    {
    }

    Cell(CellType cell_type, sf::Vector2f position);

    void set_cell_type(CellType cell_type);
    CellType get_cell_type() const;
};
} // NAMESPACE PSYNC 
#endif // !__PATH_SYNC_CELL_HPP__
