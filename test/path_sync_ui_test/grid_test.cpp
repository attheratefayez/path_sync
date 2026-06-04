#include <gtest/gtest.h>
#include "path_sync_ui/grid.hpp"
#include "path_sync_core/map_loader/map_data.hpp"

using namespace path_sync;

static MapData make_test_map(int width, int height)
{
    MapInfo info;
    info.width = width;
    info.height = height;
    info.map_name = "test";
    info.map << std::string(width * height, '.');
    return MapData(info);
}

TEST(GridTest, ConstructFromMapData)
{
    auto md = make_test_map(10, 10);
    Grid g(md, 5);

    EXPECT_EQ(g.get_width(), 10);
    EXPECT_EQ(g.get_height(), 10);
    EXPECT_EQ(g.get_cell_size(), 5);
}

TEST(GridTest, CellAccessorReturnsCorrectType)
{
    auto md = make_test_map(5, 5);
    Grid g(md, 5);

    // All cells should be DEFAULT initially
    for (int y = 0; y < g.get_height(); ++y)
        for (int x = 0; x < g.get_width(); ++x)
            EXPECT_EQ(g.get_cell(x, y).type, CellType::DEFAULT);
}

TEST(GridTest, SyncUpdatesCells)
{
    auto md = make_test_map(3, 3);
    Grid g(md, 5);

    md.set_cell_type(Coordinate(1, 1), CellType::WALL);
    g.sync_with_map_data(md);

    EXPECT_EQ(g.get_cell(1, 1).type, CellType::WALL);
    EXPECT_EQ(g.get_cell(0, 0).type, CellType::DEFAULT);
}

TEST(GridTest, CellSizeAutoAdjust)
{
    auto md = make_test_map(200, 200);
    Grid g(md, 2);

    // The grid fits inside a 1850x950 viewport
    EXPECT_GE(g.get_cell_size(), 2);
    EXPECT_LE(g.get_cell_size() * g.get_width(), 1850);
    EXPECT_LE(g.get_cell_size() * g.get_height(), 950);
}

TEST(GridTest, EmptyGridDimensions)
{
    Grid g;
    EXPECT_EQ(g.get_width(), 0);
    EXPECT_EQ(g.get_height(), 0);
    EXPECT_EQ(g.get_cell_size(), 5);
}
