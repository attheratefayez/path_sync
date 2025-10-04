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
    EXPECT_THROW(map_manager.get_current_map_data(), std::runtime_error);
    EXPECT_THROW(map_manager.get_current_scene(), std::runtime_error);
    //
    // EXPECT_NO_THROW(map_manager.get_next_map_data());
    // EXPECT_THROW(map_manager.get_current_scene(), std::runtime_error);
    //
    // EXPECT_NO_THROW(map_manager.get_next_scene(3));
    // EXPECT_NO_THROW(map_manager.get_current_scene());
    //
    // EXPECT_EQ(map_manager.get_current_scene().first.size(), 3);
}

TEST_F(MapManagerTest, MapLoading)
{
    EXPECT_THROW(map_manager.get_current_map_data(), std::runtime_error);
    EXPECT_NO_THROW(map_manager.get_next_map_data());
    EXPECT_NO_THROW(map_manager.get_current_map_data());
}

TEST_F(MapManagerTest, SceneLoading)
{
    EXPECT_NO_THROW(map_manager.get_next_map_data());
    EXPECT_THROW(map_manager.get_current_scene(), std::runtime_error);
    EXPECT_NO_THROW(map_manager.get_next_scene(5));

    EXPECT_EQ(map_manager.get_current_scene().first.size(), 5);
}

TEST_F(MapManagerTest, Reset)
{
    EXPECT_NO_THROW(map_manager.get_next_map_data());
    EXPECT_NO_THROW(map_manager.get_next_scene(2));

    EXPECT_NO_THROW(map_manager.reset());

    EXPECT_THROW(map_manager.get_current_map_data(), std::runtime_error);
    EXPECT_THROW(map_manager.get_current_scene(), std::runtime_error);
}
