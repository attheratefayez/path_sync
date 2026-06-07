#include <gtest/gtest.h>
#include <stdexcept>

#include "path_sync_core/map_loader/map_manager.hpp"

class MapManagerTest : public testing::Test 
{
protected:
    path_sync::MapManager map_manager{};
};

TEST_F(MapManagerTest, ConstructorInitialization)
{
    EXPECT_NO_THROW(map_manager.get_current_map_data());
    EXPECT_NO_THROW(map_manager.get_current_scene());
}

TEST_F(MapManagerTest, MapLoading)
{
    EXPECT_NO_THROW(map_manager.get_current_map_data());
    EXPECT_NO_THROW(map_manager.get_next_map_data());
    EXPECT_NO_THROW(map_manager.get_current_map_data());
}

TEST_F(MapManagerTest, SceneLoading)
{
    EXPECT_NO_THROW(map_manager.get_next_map_data());
    EXPECT_NO_THROW(map_manager.get_current_scene());
    EXPECT_NO_THROW(map_manager.get_next_scene(5));

    auto scene = map_manager.get_current_scene();
    if (!scene.first.empty())
        EXPECT_EQ(scene.first.size(), 5);
}

TEST_F(MapManagerTest, Reset)
{
    EXPECT_NO_THROW(map_manager.get_next_map_data());
    EXPECT_NO_THROW(map_manager.get_next_scene(2));

    EXPECT_NO_THROW(map_manager.reset());

    EXPECT_NO_THROW(map_manager.get_current_map_data());
    EXPECT_NO_THROW(map_manager.get_current_scene());
}

TEST_F(MapManagerTest, SetMapIndex)
{
    int total = map_manager.get_total_maps();
    ASSERT_GT(total, 0);

    // Load first map first
    EXPECT_NO_THROW(map_manager.get_next_map_data());

    // Jump to last map
    EXPECT_NO_THROW(map_manager.set_map_index(total - 1));
    EXPECT_EQ(map_manager.get_current_map_index(), total - 1);
    EXPECT_NO_THROW(map_manager.get_current_map_data());

    // Invalid index returns empty
    auto empty = map_manager.set_map_index(total);
    EXPECT_EQ(empty.get_height(), 0);
    empty = map_manager.set_map_index(-1);
    EXPECT_EQ(empty.get_height(), 0);
}

TEST_F(MapManagerTest, GetTotalMaps)
{
    int total = map_manager.get_total_maps();
    EXPECT_GT(total, 0);
}

TEST_F(MapManagerTest, GetPrevMapData)
{
    // Load two maps
    EXPECT_NO_THROW(map_manager.get_next_map_data());
    int idx1 = map_manager.get_current_map_index();

    EXPECT_NO_THROW(map_manager.get_next_map_data());
    int idx2 = map_manager.get_current_map_index();
    EXPECT_GT(idx2, idx1);

    // Go back
    EXPECT_NO_THROW(map_manager.get_prev_map_data());
    EXPECT_EQ(map_manager.get_current_map_index(), idx1);
}

TEST_F(MapManagerTest, SetSceneIndex)
{
    EXPECT_NO_THROW(map_manager.get_next_map_data());
    int total = map_manager.get_total_scenes();
    ASSERT_GT(total, 0);

    // Jump to last valid scene block (1 agent)
    int last_block_start = total - 1;
    if (last_block_start >= 0)
    {
        auto scene = map_manager.set_scene_index(last_block_start, 1);
        EXPECT_FALSE(scene.first.empty());
        EXPECT_EQ(scene.first.size(), 1);
    }

    // Invalid index returns empty
    auto scene = map_manager.set_scene_index(total, 1);
    EXPECT_TRUE(scene.first.empty());
    scene = map_manager.set_scene_index(-1, 1);
    EXPECT_TRUE(scene.first.empty());
}

TEST_F(MapManagerTest, SetSceneIndexMultipleAgents)
{
    EXPECT_NO_THROW(map_manager.get_next_map_data());
    int total = map_manager.get_total_scenes();
    ASSERT_GT(total, 2);

    auto scene = map_manager.set_scene_index(0, 2);
    EXPECT_FALSE(scene.first.empty());
    EXPECT_EQ(scene.first.size(), 2);
}
