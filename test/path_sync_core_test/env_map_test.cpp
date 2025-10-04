#include <gtest/gtest.h>

#include "path_sync_core/map_loader/env_map.hpp"

class MapTest : public testing::Test
{
protected:
    path_sync::Map def_constructed;
    path_sync::Map arena2_map{"arena2.map"};
    path_sync::Map arena2_map_without_ext{"arena2"};
};

TEST_F(MapTest, DefaultConstructorCreatesEmptyMap)
{
    EXPECT_TRUE(def_constructed.get_map_info().map_name.empty());
    EXPECT_TRUE(def_constructed.get_map_info().map.str().empty());
    EXPECT_EQ(def_constructed.get_map_info().height, 0);
    EXPECT_EQ(def_constructed.get_map_info().width, 0);

    EXPECT_TRUE(def_constructed.get_map_scenes().get_scene_data().empty());
}

TEST_F(MapTest, ParameterizedConstructorLoadsMap)
{
    EXPECT_EQ(arena2_map_without_ext.get_map_info().map_name, "arena2.map");
    EXPECT_EQ(arena2_map.get_map_info().map_name, "arena2.map");
    EXPECT_EQ(arena2_map.get_map_info().height, 209);
    EXPECT_EQ(arena2_map.get_map_info().width, 281);

    EXPECT_FALSE(arena2_map.get_map_scenes().get_scene_data().empty());
}

TEST_F(MapTest, MoveAssignmentMovesData)
{
    def_constructed = std::move(arena2_map);

    // checking if moved elements are in new object
    // arena2_map.map_info will still exist because MapInfo doesn't have 
    // any move constructor/move assignment operator. It uses copy 
    // for moving
    EXPECT_EQ(def_constructed.get_map_info().map_name, "arena2.map");
    EXPECT_EQ(def_constructed.get_map_info().height, 209);
    EXPECT_EQ(def_constructed.get_map_info().width, 281);

    // arena2_map.scenes will be moved as Scene has move-constructor and 
    // move assignment defined
    EXPECT_TRUE(arena2_map.get_map_scenes().get_scene_data().empty());
    EXPECT_FALSE(def_constructed.get_map_scenes().get_scene_data().empty());

}

TEST_F(MapTest, MoveConstructorMovesData)
{
    path_sync::Map moved_map(std::move(arena2_map));

    // checking if moved elements are in new object
    // arena2_map.map_info will still exist because MapInfo doesn't have 
    // any move constructor/move assignment operator. It uses copy 
    // for moving
    EXPECT_EQ(moved_map.get_map_info().map_name, "arena2.map");
    EXPECT_EQ(moved_map.get_map_info().height, 209);
    EXPECT_EQ(moved_map.get_map_info().width, 281);

    // arena2_map.scenes will be moved as Scene has move-constructor and 
    // move assignment defined
    EXPECT_TRUE(arena2_map.get_map_scenes().get_scene_data().empty());
    EXPECT_FALSE(moved_map.get_map_scenes().get_scene_data().empty());
}

