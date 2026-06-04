#include <gtest/gtest.h>

#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/path_finder.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/performance_mat.hpp"
#include "path_sync_core/solvers/astar_joint_state.hpp"
#include "path_sync_core/solvers/astar_solver.hpp"
#include "path_sync_core/solvers/bfs_solver.hpp"
#include "path_sync_core/solvers/jps_solver.hpp"
#include "path_sync_core/solvers/theta_star_solver.hpp"
#include "path_sync_core/solvers/hpa_solver.hpp"
#include "path_sync_core/solvers/dstar_lite_solver.hpp"
#include "path_sync_core/solvers/epea_solver.hpp"
#include "path_sync_core/solvers/mstar_solver.hpp"
#include "path_sync_core/solvers/astar_joint_state_utils.hpp"

namespace ps = path_sync;

class SolverTest : public testing::Test
{
  protected:
    ps::MapInfo make_map(int w, int h, std::vector<std::string> grid)
    {
        ps::MapInfo info;
        info.width = w;
        info.height = h;
        info.map_name = "test_map";
        for (auto &row : grid)
            info.map << row << "\n";
        return info;
    }
};

// --- A* Solver ---

TEST_F(SolverTest, AstarFindsPathOnOpenGrid)
{
    auto info = make_map(3, 3, {"...", "...", "..."});
    ps::MapData map(info);
    ps::solvers::sapf::Astar_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {2, 2}, perf);

    EXPECT_FALSE(result.empty());
    EXPECT_TRUE(result.contains({0, 0}));
    EXPECT_TRUE(result.contains({2, 2}));
}

TEST_F(SolverTest, AstarReturnsEmptyForUnreachableGoal)
{
    auto info = make_map(1, 1, {"#"});
    ps::MapData map(info);
    ps::solvers::sapf::Astar_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {0, 0}, perf);

    EXPECT_TRUE(result.empty());
}

// --- BFS Solver ---

TEST_F(SolverTest, BfsFindsPathOnOpenGrid)
{
    auto info = make_map(3, 3, {"...", "...", "..."});
    ps::MapData map(info);
    ps::solvers::sapf::BFS_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {2, 2}, perf);

    EXPECT_FALSE(result.empty());
    EXPECT_TRUE(result.contains({0, 0}));
    EXPECT_TRUE(result.contains({2, 2}));
}

TEST_F(SolverTest, BfsReturnsEmptyForUnreachableGoal)
{
    auto info = make_map(1, 1, {"#"});
    ps::MapData map(info);
    ps::solvers::sapf::BFS_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {0, 0}, perf);

    EXPECT_TRUE(result.empty());
}

// --- Multi-Agent Joint State Solver ---

TEST_F(SolverTest, JointStateFindsPathsForTwoAgents)
{
    auto info = make_map(3, 3, {"...", "...", "..."});
    ps::MapData map(info);
    ps::solvers::mapf::Astar_Joint_State_Solver solver;
    ps::PerformanceMetrics perf;

    std::vector<ps::Coordinate> starts = {{0, 0}, {2, 0}};
    std::vector<ps::Coordinate> goals = {{2, 2}, {0, 2}};

    auto result = solver.solve(map, starts, goals, perf);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 2);
    EXPECT_FALSE(result->at(0).empty());
    EXPECT_FALSE(result->at(1).empty());
    EXPECT_EQ(result->at(0).back(), goals[0]);
    EXPECT_EQ(result->at(1).back(), goals[1]);
}

TEST_F(SolverTest, JointStateAvoidsCollisions)
{
    auto info = make_map(2, 2, {"..", ".."});
    ps::MapData map(info);
    ps::solvers::mapf::Astar_Joint_State_Solver solver;
    ps::PerformanceMetrics perf;

    std::vector<ps::Coordinate> starts = {{0, 0}, {1, 1}};
    std::vector<ps::Coordinate> goals = {{1, 1}, {0, 0}};

    auto result = solver.solve(map, starts, goals, perf);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 2);
    EXPECT_EQ(result->at(0).back(), goals[0]);
    EXPECT_EQ(result->at(1).back(), goals[1]);
}

TEST_F(SolverTest, JointStateReturnsNulloptForMismatchedCounts)
{
    auto info = make_map(3, 3, {"...", "...", "..."});
    ps::MapData map(info);
    ps::solvers::mapf::Astar_Joint_State_Solver solver;
    ps::PerformanceMetrics perf;

    std::vector<ps::Coordinate> starts = {{0, 0}, {1, 0}};
    std::vector<ps::Coordinate> goals = {{2, 2}};

    auto result = solver.solve(map, starts, goals, perf);

    EXPECT_FALSE(result.has_value());
}

// --- PathFinder Dispatch ---

TEST_F(SolverTest, PathFinderFindsSingleAgentPath)
{
    auto info = make_map(3, 3, {"...", "...", "..."});
    ps::MapData map(info);
    ps::PathFinder finder;

    std::vector<ps::Coordinate> starts = {{0, 0}};
    std::vector<ps::Coordinate> ends = {{2, 2}};

    auto result = finder.find_path(map, starts, ends);

    ASSERT_TRUE(std::holds_alternative<std::vector<ps::Coordinate>>(result));
    auto path = std::get<std::vector<ps::Coordinate>>(result);
    EXPECT_FALSE(path.empty());
    EXPECT_EQ(path.front(), starts[0]);
    EXPECT_EQ(path.back(), ends[0]);
}

TEST_F(SolverTest, PathFinderFindsMultiAgentPath)
{
    auto info = make_map(3, 3, {"...", "...", "..."});
    ps::MapData map(info);
    ps::PathFinder finder;

    std::vector<ps::Coordinate> starts = {{0, 0}, {2, 0}};
    std::vector<ps::Coordinate> ends = {{2, 2}, {0, 2}};

    auto result = finder.find_path(map, starts, ends);

    ASSERT_TRUE(std::holds_alternative<std::vector<std::vector<ps::Coordinate>>>(result));
    auto paths = std::get<std::vector<std::vector<ps::Coordinate>>>(result);
    ASSERT_EQ(paths.size(), 2);
    EXPECT_FALSE(paths[0].empty());
    EXPECT_FALSE(paths[1].empty());
    EXPECT_EQ(paths[0].back(), ends[0]);
    EXPECT_EQ(paths[1].back(), ends[1]);
}

TEST_F(SolverTest, PathFinderThrowsOnMismatchedCounts)
{
    auto info = make_map(3, 3, {"...", "...", "..."});
    ps::MapData map(info);
    ps::PathFinder finder;

    std::vector<ps::Coordinate> starts = {{0, 0}, {1, 0}};
    std::vector<ps::Coordinate> ends = {{2, 2}};

    EXPECT_THROW(static_cast<void>(finder.find_path(map, starts, ends)), std::logic_error);
}

TEST_F(SolverTest, ChangeSolverCyclesAllSolvers)
{
    ps::PathFinder finder;

    std::string_view first = finder.get_current_solver_name();
    std::string_view prev = first;
    int count = 1;

    for (int i = 0; i < 20; i++)
    {
        finder.change_solver();
        std::string_view cur = finder.get_current_solver_name();
        if (cur == first) break;
        EXPECT_NE(cur, prev);
        prev = cur;
        count++;
    }

    EXPECT_EQ(finder.get_current_solver_name(), first);
    EXPECT_GT(count, 4); // at least 5+ solvers now
}

// --- JPS Solver ---

TEST_F(SolverTest, JpsFindsPathOnOpenGrid)
{
    auto info = make_map(3, 3, {"...", "...", "..."});
    ps::MapData map(info);
    ps::solvers::sapf::JPS_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {2, 2}, perf);

    EXPECT_FALSE(result.empty());
    EXPECT_TRUE(result.contains({0, 0}));
    EXPECT_TRUE(result.contains({2, 2}));
}

TEST_F(SolverTest, JpsReturnsEmptyForUnreachableGoal)
{
    auto info = make_map(1, 1, {"#"});
    ps::MapData map(info);
    ps::solvers::sapf::JPS_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {0, 0}, perf);

    EXPECT_TRUE(result.empty());
}

// --- Theta* Solver ---

TEST_F(SolverTest, ThetaStarFindsPathOnOpenGrid)
{
    auto info = make_map(3, 3, {"...", "...", "..."});
    ps::MapData map(info);
    ps::solvers::sapf::Theta_Star_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {2, 2}, perf);

    EXPECT_FALSE(result.empty());
    EXPECT_TRUE(result.contains({0, 0}));
    EXPECT_TRUE(result.contains({2, 2}));
}

TEST_F(SolverTest, ThetaStarReturnsEmptyForUnreachableGoal)
{
    auto info = make_map(1, 1, {"#"});
    ps::MapData map(info);
    ps::solvers::sapf::Theta_Star_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {0, 0}, perf);

    EXPECT_TRUE(result.empty());
}

// --- HPA* Solver ---

TEST_F(SolverTest, HpaFindsPathOnOpenGrid)
{
    auto info = make_map(6, 6, {"......", "......", "......", "......", "......", "......"});
    ps::MapData map(info);
    ps::solvers::sapf::HPA_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {5, 5}, perf);

    EXPECT_FALSE(result.empty());
    EXPECT_TRUE(result.contains({0, 0}));
    EXPECT_TRUE(result.contains({5, 5}));
}

TEST_F(SolverTest, HpaReturnsEmptyForUnreachableGoal)
{
    auto info = make_map(1, 1, {"#"});
    ps::MapData map(info);
    ps::solvers::sapf::HPA_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {0, 0}, perf);

    EXPECT_TRUE(result.empty());
}

// --- D* Lite Solver ---

TEST_F(SolverTest, DStarLiteFindsPathOnOpenGrid)
{
    auto info = make_map(3, 3, {"...", "...", "..."});
    ps::MapData map(info);
    ps::solvers::sapf::DStar_Lite_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {2, 2}, perf);

    EXPECT_FALSE(result.empty());
    EXPECT_TRUE(result.contains({0, 0}));
    EXPECT_TRUE(result.contains({2, 2}));
}

TEST_F(SolverTest, DStarLiteReturnsEmptyForUnreachableGoal)
{
    auto info = make_map(1, 1, {"#"});
    ps::MapData map(info);
    ps::solvers::sapf::DStar_Lite_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {0, 0}, perf);

    EXPECT_TRUE(result.empty());
}

// --- EPEA* Solver ---

TEST_F(SolverTest, EpeaStarFindsPathOnOpenGrid)
{
    auto info = make_map(3, 3, {"...", "...", "..."});
    ps::MapData map(info);
    ps::solvers::sapf::EPEA_Star_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {2, 2}, perf);

    EXPECT_FALSE(result.empty());
    EXPECT_TRUE(result.contains({0, 0}));
    EXPECT_TRUE(result.contains({2, 2}));
}

TEST_F(SolverTest, EpeaStarReturnsEmptyForUnreachableGoal)
{
    auto info = make_map(1, 1, {"#"});
    ps::MapData map(info);
    ps::solvers::sapf::EPEA_Star_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {0, 0}, perf);

    EXPECT_TRUE(result.empty());
}

// --- M* Solver ---

TEST_F(SolverTest, MStarFindsPathsForTwoAgents)
{
    auto info = make_map(3, 3, {"...", "...", "..."});
    ps::MapData map(info);
    ps::solvers::mapf::MStar_Solver solver;
    ps::PerformanceMetrics perf;

    std::vector<ps::Coordinate> starts = {{0, 0}, {2, 0}};
    std::vector<ps::Coordinate> goals = {{2, 2}, {0, 2}};

    auto result = solver.solve(map, starts, goals, perf);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 2);
    EXPECT_FALSE(result->at(0).empty());
    EXPECT_FALSE(result->at(1).empty());
    EXPECT_EQ(result->at(0).back(), goals[0]);
    EXPECT_EQ(result->at(1).back(), goals[1]);
}

TEST_F(SolverTest, MStarReturnsNulloptForMismatchedCounts)
{
    auto info = make_map(3, 3, {"...", "...", "..."});
    ps::MapData map(info);
    ps::solvers::mapf::MStar_Solver solver;
    ps::PerformanceMetrics perf;

    std::vector<ps::Coordinate> starts = {{0, 0}, {1, 0}};
    std::vector<ps::Coordinate> goals = {{2, 2}};

    auto result = solver.solve(map, starts, goals, perf);

    EXPECT_FALSE(result.has_value());
}

// --- Utils ---

TEST_F(SolverTest, CartesianProductWithSingleInput)
{
    std::vector<ps::Coordinate> moves = {{0, 0}, {1, 0}};
    std::vector<std::vector<ps::Coordinate>> input = {moves};

    auto result = mapf::astar_joint_state::Utils::cartesian_product(input);

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->size(), 2);
    EXPECT_EQ(result->at(0), std::vector<ps::Coordinate>({{0, 0}}));
    EXPECT_EQ(result->at(1), std::vector<ps::Coordinate>({{1, 0}}));
}

TEST_F(SolverTest, CartesianProductWithEmptyInput)
{
    std::vector<std::vector<ps::Coordinate>> input;

    auto result = mapf::astar_joint_state::Utils::cartesian_product(input);

    EXPECT_FALSE(result.has_value());
}

TEST_F(SolverTest, HeuristicComputesCorrectly)
{
    std::vector<ps::Coordinate> starts = {{0, 0}, {2, 0}};
    std::vector<ps::Coordinate> goals = {{2, 2}, {0, 2}};

    float h = mapf::astar_joint_state::Utils::heuristic(starts, goals);

    float expected = 4 + 4;
    EXPECT_FLOAT_EQ(h, expected);
}

TEST_F(SolverTest, CheckValidityDetectsVertexConflict)
{
    ps::mapf_type::JointState current;
    current.positions = {{0, 0}, {1, 1}};

    ps::mapf_type::JointState next;
    next.positions = {{1, 1}, {1, 1}};

    EXPECT_FALSE(mapf::astar_joint_state::Utils::check_validity_of_state(current, next));
}

TEST_F(SolverTest, CheckValidityDetectsEdgeConflict)
{
    ps::mapf_type::JointState current;
    current.positions = {{0, 0}, {1, 1}};

    ps::mapf_type::JointState next;
    next.positions = {{1, 1}, {0, 0}};

    EXPECT_FALSE(mapf::astar_joint_state::Utils::check_validity_of_state(current, next));
}

TEST_F(SolverTest, ExtractPathsBuildsCorrectPaths)
{
    auto initial_node = std::make_shared<ps::mapf_type::Node>(
        ps::mapf_type::JointState{{{0, 0}, {2, 0}}, 0}, 0, 0);
    auto mid_node = std::make_shared<ps::mapf_type::Node>(
        ps::mapf_type::JointState{{{1, 1}, {1, 1}}, 1}, 0, 0, initial_node);
    auto goal_node = std::make_shared<ps::mapf_type::Node>(
        ps::mapf_type::JointState{{{2, 2}, {0, 2}}, 2}, 0, 0, mid_node);

    auto paths = mapf::astar_joint_state::Utils::extract_paths(goal_node);

    ASSERT_EQ(paths.size(), 2);
    ASSERT_EQ(paths[0].size(), 3);
    ASSERT_EQ(paths[1].size(), 3);
    EXPECT_EQ(paths[0][0], (ps::Coordinate{0, 0}));
    EXPECT_EQ(paths[0][2], (ps::Coordinate{2, 2}));
    EXPECT_EQ(paths[1][0], (ps::Coordinate{2, 0}));
    EXPECT_EQ(paths[1][2], (ps::Coordinate{0, 2}));
}
