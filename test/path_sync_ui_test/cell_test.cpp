#include <gtest/gtest.h>
#include "path_sync_ui/cell.hpp"

using namespace path_sync;

TEST(CellTest, DefaultConstructor)
{
    Cell c;
    EXPECT_EQ(c.type, CellType::DEFAULT);
    EXPECT_EQ(c.grid_x, 0);
    EXPECT_EQ(c.grid_y, 0);
}

TEST(CellTest, ParameterizedConstructor)
{
    Cell c(CellType::WALL, 3, 7);
    EXPECT_EQ(c.type, CellType::WALL);
    EXPECT_EQ(c.grid_x, 3);
    EXPECT_EQ(c.grid_y, 7);
}

TEST(CellTest, SetAndGetCellType)
{
    Cell c(CellType::DEFAULT, 0, 0);
    EXPECT_EQ(c.get_cell_type(), CellType::DEFAULT);

    c.set_cell_type(CellType::START);
    EXPECT_EQ(c.get_cell_type(), CellType::START);

    c.set_cell_type(CellType::END);
    EXPECT_EQ(c.get_cell_type(), CellType::END);

    c.set_cell_type(CellType::PATH);
    EXPECT_EQ(c.get_cell_type(), CellType::PATH);
}

TEST(CellTest, SetCellTypeUpdatesMemberDirectly)
{
    Cell c;
    c.type = CellType::WALL;
    EXPECT_EQ(c.get_cell_type(), CellType::WALL);
}
