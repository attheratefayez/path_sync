#include <gtest/gtest.h>
#include <vector>

#include "path_sync/path_sync_types.hpp"
#include "path_sync/solvers/astar_joint_state_utils.hpp"

using namespace mapf::astar_joint_state;

class AstarJointStateUtilTest : public testing::Test
{
  protected:
    AstarJointStateUtilTest() {};

    Coordinate p1 = {0, 1};
    Coordinate p2 = {2, 3};
    Coordinate p3 = {4, 5};
    Coordinate p4 = {1, 3};
    Coordinate p5 = {1, 4};
    Coordinate p6 = {6, 9};

    std::vector<Coordinate> first_list = {p1, p2, p3};
    std::vector<Coordinate> second_list = {p4, p5, p6};

    std::vector<std::vector<Coordinate>> empty_mixed_list;
    std::vector<std::vector<Coordinate>> mixed_list = {{p1, p2}, {p3, p4}};
    std::vector<std::vector<Coordinate>> mixed_list_cp = {{p1, p3}, {p2, p3}, {p1, p4}, {p2, p4}};
};

// MANHATTAN_DISTANCE
TEST_F(AstarJointStateUtilTest, Manhattan_Distance_ZeroDistance)
{
    EXPECT_EQ(Utils::manhattan_distance(Coordinate(0, 0), Coordinate(0, 0)), 0);
}

TEST_F(AstarJointStateUtilTest, Manhattan_Distance_GeneralCase)
{
    EXPECT_EQ(Utils::manhattan_distance(Coordinate(1, 1), Coordinate(1, 2)), 1);
    EXPECT_EQ(Utils::manhattan_distance(Coordinate(4, 2), Coordinate(7, 9)), 10);
}

// HEURISTIC
TEST_F(AstarJointStateUtilTest, Heuristic_EmptyVector)
{
    EXPECT_EQ(Utils::heuristic(std::vector<Coordinate>{}, std::vector<Coordinate>{}), 0);
}

TEST_F(AstarJointStateUtilTest, Heuristic_GenerealCase)
{
    EXPECT_EQ(Utils::heuristic(first_list, second_list), 11);
}

// CARTESIAN PRODUCT
TEST_F(AstarJointStateUtilTest, CartesianProduct_EmptyVector)
{
    EXPECT_EQ(Utils::cartesian_product(empty_mixed_list), std::nullopt);
}

TEST_F(AstarJointStateUtilTest, CartesianProduct_GeneralCase)
{
    EXPECT_EQ(Utils::cartesian_product(mixed_list), mixed_list_cp);
}
