#include <gtest/gtest.h>
#include "path_sync_ui/cell.hpp"

using namespace path_sync;

TEST(ColorTest, AllCellTypesReturnNonDefaultColor)
{
    EXPECT_EQ(cell_type_to_color(CellType::DEFAULT), QColor(255, 255, 255));
    EXPECT_EQ(cell_type_to_color(CellType::WALL),    QColor(39, 55, 70));
    EXPECT_EQ(cell_type_to_color(CellType::FOUND),   QColor(0, 255, 0));
    EXPECT_EQ(cell_type_to_color(CellType::VISITED), QColor(39, 245, 71, 80));
    EXPECT_EQ(cell_type_to_color(CellType::START),   QColor(44, 235, 6));
    EXPECT_EQ(cell_type_to_color(CellType::END),     QColor(255, 0, 0));
    EXPECT_EQ(cell_type_to_color(CellType::PATH),    QColor(0, 0, 255));
}

TEST(ColorTest, DefaultColorMatchesWhite)
{
    QColor white = cell_type_to_color(CellType::DEFAULT);
    EXPECT_TRUE(white.isValid());
    EXPECT_EQ(white.red(), 255);
    EXPECT_EQ(white.green(), 255);
    EXPECT_EQ(white.blue(), 255);
}

TEST(ColorTest, WallColorIsDark)
{
    QColor wall = cell_type_to_color(CellType::WALL);
    EXPECT_TRUE(wall.isValid());
    // All channels should be relatively dark (< 100)
    EXPECT_LT(wall.red(), 100);
    EXPECT_LT(wall.green(), 100);
    EXPECT_LT(wall.blue(), 100);
}
