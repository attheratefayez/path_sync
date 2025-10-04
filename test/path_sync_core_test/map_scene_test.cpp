#include <filesystem>
#include <gtest/gtest.h>

#include "path_sync_core/map_loader/map_scene.hpp"

class SceneTest : public testing::Test 
{
protected:
    path_sync::Scene def_scene;
    path_sync::Scene without_ext{"arena2"};
    path_sync::Scene with_ext{"arena2.map"};

};

TEST_F(SceneTest, DefaultConstructorCreatesEmptyScene)
{
    EXPECT_TRUE(def_scene.get_scene_data().empty());
}

TEST_F(SceneTest, NoSuchFile)
{
    EXPECT_THROW(path_sync::Scene("no_file"), std::filesystem::filesystem_error);
}

TEST_F(SceneTest, CheckFileRead)
{
    EXPECT_EQ(without_ext.get_scene_data().size(), with_ext.get_scene_data().size());
}

TEST_F(SceneTest, CheckSceneDataExists)
{
    EXPECT_GT(with_ext.get_scene_data().size(), 0);
}

TEST_F(SceneTest, MoveConstructorMovesData)
{
    std::size_t temp_size = without_ext.get_scene_data().size();
    path_sync::Scene move_constructed(std::move(without_ext));

    EXPECT_EQ(move_constructed.get_scene_data().size(), temp_size);
    EXPECT_TRUE(without_ext.get_scene_data().empty());
}

TEST_F(SceneTest, MoveAssignmentMovesData)
{
    std::size_t temp_size = without_ext.get_scene_data().size();
    def_scene = std::move(without_ext);

    EXPECT_EQ(def_scene.get_scene_data().size(), temp_size);
    EXPECT_TRUE(without_ext.get_scene_data().empty());
}
