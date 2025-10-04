#include <gtest/gtest.h>

#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/map_loader/env_map.hpp"

class MapDataTest : public testing::Test 
{
protected:
    path_sync::Map arena2_map{"arena2"};

    path_sync::MapData def_constructed;
    path_sync::MapData arena2_map_data{arena2_map.get_map_info()};
};

TEST_F(MapDataTest, DefaultConstructorCreatesEmptyMapData)
{
    EXPECT_EQ(def_constructed.get_height(), 0);
    EXPECT_EQ(def_constructed.get_width(), 0);
}

TEST_F(MapDataTest, ParameterizedConstructorLoadsMapData)
{
    EXPECT_EQ(arena2_map_data.get_height(), 209);
    EXPECT_EQ(arena2_map_data.get_width(), 281);
}

TEST_F(MapDataTest, SetterGetterTest)
{
    arena2_map_data.set_cell_type({150, 150}, path_sync::CellType::START);
    EXPECT_EQ(arena2_map_data.get_cell_type({150, 150}), path_sync::CellType::START);

    arena2_map_data.set_cell_type({150, 150}, path_sync::CellType::DEFAULT);
    EXPECT_EQ(arena2_map_data.get_cell_type({150, 150}), path_sync::CellType::DEFAULT);


}
