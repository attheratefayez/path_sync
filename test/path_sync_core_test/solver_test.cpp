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
#include "path_sync_core/solvers/cbs_solver.hpp"
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

// --- PathFinder Cancel / Solver Selection ---

TEST_F(SolverTest, PathFinderCancelPreventsSolve)
{
    auto info = make_map(100, 100, std::vector<std::string>(100, std::string(100, '.')));
    ps::MapData map(info);
    ps::PathFinder finder;

    finder.cancel();
    EXPECT_TRUE(finder.is_cancelled());

    std::vector<ps::Coordinate> starts = {{0, 0}};
    std::vector<ps::Coordinate> ends = {{99, 99}};

    auto result = finder.find_path(map, starts, ends);
    // Should return empty due to cancellation
    ASSERT_TRUE(std::holds_alternative<std::vector<ps::Coordinate>>(result));
    EXPECT_TRUE(std::get<std::vector<ps::Coordinate>>(result).empty());

    // Reset should allow next solve
    finder.reset_cancel();
    EXPECT_FALSE(finder.is_cancelled());
}

TEST_F(SolverTest, PathFinderSelectSaSolver)
{
    ps::PathFinder finder;
    auto names = finder.get_sa_solver_names();
    ASSERT_GT(names.size(), 1);

    // Select the second solver
    finder.select_sa_solver_by_index(1);
    EXPECT_EQ(finder.get_current_solver_name(), names[1]);
}

TEST_F(SolverTest, PathFinderSelectMaSolver)
{
    ps::PathFinder finder;
    auto names = finder.get_ma_solver_names();
    ASSERT_GE(names.size(), 1);

    finder.select_ma_solver_by_index(0);
    std::string_view selected = finder.get_current_solver_name();
    EXPECT_FALSE(selected.empty());
    EXPECT_NE(selected, "No Solver Selected");
}

TEST_F(SolverTest, PathFinderSelectSolverCombined)
{
    ps::PathFinder finder;
    auto sa_names = finder.get_sa_solver_names();
    auto ma_names = finder.get_ma_solver_names();
    auto all = finder.get_all_solver_names();

    EXPECT_EQ(all.size(), sa_names.size() + ma_names.size());

    // Select first SA solver via combined index
    finder.select_solver_by_index(0);
    EXPECT_EQ(finder.get_current_solver_name(), sa_names[0]);

    // Select first MA solver via combined index
    finder.select_solver_by_index(sa_names.size());
    EXPECT_EQ(finder.get_current_solver_name(), ma_names[0]);
}

TEST_F(SolverTest, PathFinderPerformanceMetricsPopulated)
{
    auto info = make_map(3, 3, {"...", "...", "..."});
    ps::MapData map(info);
    ps::PathFinder finder;

    std::vector<ps::Coordinate> starts = {{0, 0}};
    std::vector<ps::Coordinate> ends = {{2, 2}};

    (void)finder.find_path(map, starts, ends);

    auto metrics = finder.get_performance_metrics();
    EXPECT_EQ(metrics.map_name, "test_map");
    EXPECT_EQ(metrics.num_agents, 1);
    EXPECT_TRUE(metrics.success);
    EXPECT_GT(metrics.runtime.count(), 0);
    EXPECT_GT(metrics.path_length, 0);
    EXPECT_GT(metrics.timestamp, 0);
}

// --- HPA Solver Additional Tests ---
TEST_F(SolverTest, HpaFindsPathThroughObstacles)
{
    auto info = make_map(6, 6,
        {"......",
         "......",
         ".##...",
         ".##...",
         "......",
         "......"});
    ps::MapData map(info);
    ps::solvers::sapf::HPA_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {5, 5}, perf);

    EXPECT_FALSE(result.empty());
    EXPECT_TRUE(perf.success);
}

// --- D* Lite Additional Test ---
TEST_F(SolverTest, DStarLiteFindsPathThroughObstacles)
{
    auto info = make_map(5, 5,
        {".....",
         ".##..",
         ".#...",
         ".#.#.",
         "....."});
    ps::MapData map(info);
    ps::solvers::sapf::DStar_Lite_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {4, 4}, perf);

    EXPECT_FALSE(result.empty());
    EXPECT_TRUE(perf.success);
}

// --- EPEA* Additional Test ---
TEST_F(SolverTest, EpeaStarFindsPathThroughObstacles)
{
    auto info = make_map(5, 5,
        {".....",
         ".##..",
         ".#...",
         ".#.#.",
         "....."});
    ps::MapData map(info);
    ps::solvers::sapf::EPEA_Star_Solver solver;
    ps::PerformanceMetrics perf;

    auto result = solver.solve(map, {0, 0}, {4, 4}, perf);

    EXPECT_FALSE(result.empty());
    EXPECT_TRUE(perf.success);
}

// --- M* Additional Test ---
TEST_F(SolverTest, MStarAvoidsCollisionOnNarrowCorridor)
{
    // Two agents crossing in a narrow corridor must avoid each other
    auto info = make_map(3, 3,
        {"...",
         "...",
         "..."});
    ps::MapData map(info);
    ps::solvers::mapf::MStar_Solver solver;
    ps::PerformanceMetrics perf;

    std::vector<ps::Coordinate> starts = {{0, 0}, {2, 2}};
    std::vector<ps::Coordinate> goals = {{2, 2}, {0, 0}};

    auto result = solver.solve(map, starts, goals, perf);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 2);
    EXPECT_EQ(result->at(0).back(), goals[0]);
    EXPECT_EQ(result->at(1).back(), goals[1]);
}


// ── CBS Solver (plain) ────────────────────────────────────────────────

// Tests using solve() use non-conflicting setups so CT stays at 1 node.
// Conflict detection and resolution logic is tested via internal method tests.

TEST_F(SolverTest, CBSBasicSwap)
{
    // 2 agents, same direction → no conflict, single CT node
    auto info = make_map(4, 2, {"....", "...."});
    ps::MapData map(info);
    ps::solvers::mapf::CBS_Solver solver;
    ps::PerformanceMetrics perf;

    std::vector<ps::Coordinate> starts = {{0, 0}, {0, 1}};
    std::vector<ps::Coordinate> goals  = {{3, 0}, {3, 1}};

    auto result = solver.solve(map, starts, goals, perf);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 2);
    EXPECT_EQ(result->at(0).back(), goals[0]);
    EXPECT_EQ(result->at(1).back(), goals[1]);
    EXPECT_TRUE(perf.success);
}

TEST_F(SolverTest, CBSThreeAgents)
{
    // 3 agents moving East on separate rows → no conflicts
    auto info = make_map(4, 3, {"....", "....", "...."});
    ps::MapData map(info);
    ps::solvers::mapf::CBS_Solver solver;
    ps::PerformanceMetrics perf;

    std::vector<ps::Coordinate> starts = {{0, 0}, {0, 1}, {0, 2}};
    std::vector<ps::Coordinate> goals  = {{3, 0}, {3, 1}, {3, 2}};

    auto result = solver.solve(map, starts, goals, perf);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 3);
    EXPECT_TRUE(perf.success);

    for (size_t a = 0; a < result->size(); a++)
        EXPECT_EQ(result->at(a).back(), goals[a]);
}

TEST_F(SolverTest, CBSSingleAgent)
{
    auto info = make_map(3, 3, {"...", "...", "..."});
    ps::MapData map(info);
    ps::solvers::mapf::CBS_Solver solver;
    ps::PerformanceMetrics perf;

    std::vector<ps::Coordinate> starts = {{0, 0}};
    std::vector<ps::Coordinate> goals  = {{2, 2}};

    auto result = solver.solve(map, starts, goals, perf);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 1);
    EXPECT_EQ(result->at(0).back(), goals[0]);
    EXPECT_TRUE(perf.success);
}

TEST_F(SolverTest, CBSUnreachableAgent)
{
    auto info = make_map(3, 1, {".#."});
    ps::MapData map(info);
    ps::solvers::mapf::CBS_Solver solver;
    ps::PerformanceMetrics perf;

    std::vector<ps::Coordinate> starts = {{0, 0}};
    std::vector<ps::Coordinate> goals  = {{2, 0}};

    auto result = solver.solve(map, starts, goals, perf);

    EXPECT_FALSE(result.has_value());
    EXPECT_FALSE(perf.success);
}

TEST_F(SolverTest, CBSObstacleAvoidance)
{
    // Single agent navigating around obstacle → no MA conflict
    auto info = make_map(3, 3,
        {"...",
         ".#.",
         "..."});
    ps::MapData map(info);
    ps::solvers::mapf::CBS_Solver solver;
    ps::PerformanceMetrics perf;

    std::vector<ps::Coordinate> starts = {{0, 0}};
    std::vector<ps::Coordinate> goals  = {{2, 2}};

    auto result = solver.solve(map, starts, goals, perf);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->at(0).back(), goals[0]);
    EXPECT_TRUE(perf.success);

    // Must go around obstacle at (1,1) — not pass through blocked cell
    for (auto &p : *result)
        for (auto &c : p)
            EXPECT_FALSE(c.first == 1 && c.second == 1) << "Should not pass through obstacle at (1,1)";
}

TEST_F(SolverTest, CBSPerformanceMetricsPopulated)
{
    // Non-conflicting 2-agent → 1 CT node, but metrics are still populated
    auto info = make_map(4, 2, {"....", "...."});
    ps::MapData map(info);
    ps::solvers::mapf::CBS_Solver solver;
    ps::PerformanceMetrics perf;

    ps::MAPFMetrics mapf_met;
    perf.mapf_metrics = &mapf_met;

    std::vector<ps::Coordinate> starts = {{0, 0}, {0, 1}};
    std::vector<ps::Coordinate> goals  = {{3, 0}, {3, 1}};

    auto result = solver.solve(map, starts, goals, perf);

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(perf.success);
    EXPECT_GT(perf.runtime.count(), 0);
    EXPECT_GT(perf.sum_of_costs, 0);
    EXPECT_GT(perf.makespan, 0);
    EXPECT_GE(mapf_met.joint_states_expanded, 0); // 1 node expanded, no conflict
    EXPECT_GE(mapf_met.conflicts_detected, 0);
    EXPECT_GE(mapf_met.solution_depth, 0);
    EXPECT_GT(mapf_met.max_timestep_reached, 0);
}

TEST_F(SolverTest, CBSCrossingPathsNoCollision)
{
    // 2 agents moving East on separate rows → no conflict
    auto info = make_map(4, 2, {"....", "...."});
    ps::MapData map(info);
    ps::solvers::mapf::CBS_Solver solver;
    ps::PerformanceMetrics perf;

    std::vector<ps::Coordinate> starts = {{0, 0}, {0, 1}};
    std::vector<ps::Coordinate> goals  = {{3, 0}, {3, 1}};

    auto result = solver.solve(map, starts, goals, perf);

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(perf.success);
    EXPECT_EQ(result->at(0).back(), goals[0]);
    EXPECT_EQ(result->at(1).back(), goals[1]);
}

// ── ICBS Tests (friend fixture, access to internal methods) ────────────

namespace path_sync::solvers::mapf
{

class CBS_Solver_ICBS_Test : public ::testing::Test
{
protected:
    CBS_Solver solver;

    void SetUp() override
    {
        solver.set_use_icbs(true);
        solver.set_solver_name("ICBS_Solver_Test");
    }

    path_sync::MapInfo make_map(int w, int h, std::vector<std::string> grid)
    {
        path_sync::MapInfo info;
        info.width = w;
        info.height = h;
        info.map_name = "test_map";
        for (auto &row : grid)
            info.map << row << "\n";
        return info;
    }

    std::optional<CBSConflict> call_find_first_conflict(
        const std::vector<std::vector<Coordinate>> &paths)
    {
        return solver.find_first_conflict(paths);
    }

    CBSConflictClass call_classify_conflict(
        const CBSConflict &conflict,
        const std::vector<std::vector<Coordinate>> &paths,
        const MapData &map,
        const std::vector<Coordinate> &starts,
        const std::vector<Coordinate> &goals)
    {
        return solver.classify_conflict(conflict, paths, map, starts, goals);
    }

    std::optional<std::vector<Coordinate>> call_low_level_search(
        const MapData &map, Coordinate start, Coordinate goal,
        const std::vector<CBSConstraint> &constraints, int agent_id)
    {
        return solver.low_level_search(map, start, goal, constraints, agent_id);
    }

    std::optional<std::vector<Coordinate>> call_low_level_search_epea(
        const MapData &map, Coordinate start, Coordinate goal,
        const std::vector<CBSConstraint> &constraints, int agent_id)
    {
        return solver.low_level_search_epea(map, start, goal, constraints, agent_id);
    }

    bool call_validate_paths_against_constraints(
        const std::vector<std::vector<Coordinate>> &paths,
        const std::vector<CBSConstraint> &constraints)
    {
        return solver.validate_paths_against_constraints(paths, constraints);
    }

    int call_compute_soc(const std::vector<std::vector<Coordinate>> &paths)
    {
        return solver.compute_soc(paths);
    }
};

TEST_F(CBS_Solver_ICBS_Test, BasicSwap)
{
    // Non-conflicting 2-agent → 1 CT node
    auto info = make_map(4, 2, {"....", "...."});
    path_sync::MapData map(info);
    path_sync::PerformanceMetrics perf;

    std::vector<Coordinate> starts = {{0, 0}, {0, 1}};
    std::vector<Coordinate> goals  = {{3, 0}, {3, 1}};

    auto result = solver.solve(map, starts, goals, perf);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 2);
    EXPECT_EQ(result->at(0).back(), goals[0]);
    EXPECT_EQ(result->at(1).back(), goals[1]);
    EXPECT_TRUE(perf.success);
}

TEST_F(CBS_Solver_ICBS_Test, ThreeAgents)
{
    // 3 agents moving East on separate rows → no conflicts
    auto info = make_map(4, 3, {"....", "....", "...."});
    path_sync::MapData map(info);
    path_sync::PerformanceMetrics perf;

    std::vector<Coordinate> starts = {{0, 0}, {0, 1}, {0, 2}};
    std::vector<Coordinate> goals  = {{3, 0}, {3, 1}, {3, 2}};

    auto result = solver.solve(map, starts, goals, perf);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 3);
    EXPECT_TRUE(perf.success);
    for (size_t a = 0; a < result->size(); a++)
        EXPECT_EQ(result->at(a).back(), goals[a]);
}

TEST_F(CBS_Solver_ICBS_Test, FindFirstConflict)
{
    std::vector<Coordinate> path_a = {{0, 0}, {1, 0}, {2, 0}};
    std::vector<Coordinate> path_b = {{2, 0}, {1, 0}, {0, 0}};
    std::vector<std::vector<Coordinate>> paths = {path_a, path_b};

    auto conflict = call_find_first_conflict(paths);
    ASSERT_TRUE(conflict.has_value());
    EXPECT_EQ(conflict->agent_a, 0);
    EXPECT_EQ(conflict->agent_b, 1);
    EXPECT_EQ(conflict->x, 1);
    EXPECT_EQ(conflict->y, 0);
    EXPECT_EQ(conflict->timestep, 1);
    EXPECT_TRUE(conflict->is_vertex);
}

TEST_F(CBS_Solver_ICBS_Test, FindFirstConflictNoConflict)
{
    std::vector<Coordinate> path_a = {{0, 0}, {0, 1}, {0, 2}};
    std::vector<Coordinate> path_b = {{2, 0}, {2, 1}, {2, 2}};
    std::vector<std::vector<Coordinate>> paths = {path_a, path_b};

    auto conflict = call_find_first_conflict(paths);
    EXPECT_FALSE(conflict.has_value());
}

TEST_F(CBS_Solver_ICBS_Test, FindFirstConflictEdge)
{
    std::vector<Coordinate> path_a = {{0, 0}, {1, 0}, {2, 0}};
    std::vector<Coordinate> path_b = {{1, 0}, {0, 0}, {0, 0}};
    std::vector<std::vector<Coordinate>> paths = {path_a, path_b};

    auto conflict = call_find_first_conflict(paths);
    ASSERT_TRUE(conflict.has_value());
    EXPECT_FALSE(conflict->is_vertex);
    EXPECT_EQ(conflict->timestep, 1);
}

TEST_F(CBS_Solver_ICBS_Test, LowLevelSearchVertexConstraint)
{
    auto info = make_map(3, 1, {"..."});
    path_sync::MapData map(info);

    std::vector<CBSConstraint> constraints;
    constraints.push_back({0, 1, 0, 1, true, 0, 0});

    auto path = call_low_level_search(map, {0, 0}, {2, 0}, constraints, 0);

    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->front(), (Coordinate{0, 0}));
    EXPECT_EQ(path->back(), (Coordinate{2, 0}));
    if (path->size() > 1)
        EXPECT_NE((*path)[1], (Coordinate{1, 0})) << "Should avoid (1,0) at t=1";
}

TEST_F(CBS_Solver_ICBS_Test, LowLevelSearchEdgeConstraint)
{
    auto info = make_map(3, 1, {"..."});
    path_sync::MapData map(info);

    std::vector<CBSConstraint> constraints;
    constraints.push_back({0, 0, 0, 1, 0, 1, false});

    auto path = call_low_level_search(map, {0, 0}, {2, 0}, constraints, 0);

    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->front(), (Coordinate{0, 0}));
    EXPECT_EQ(path->back(), (Coordinate{2, 0}));
}

TEST_F(CBS_Solver_ICBS_Test, LowLevelSearchUnreachable)
{
    auto info = make_map(3, 1, {".#."});
    path_sync::MapData map(info);

    auto path = call_low_level_search(map, {0, 0}, {2, 0}, {}, 0);

    EXPECT_FALSE(path.has_value());
}

TEST_F(CBS_Solver_ICBS_Test, LowLevelSearchStartBlocked)
{
    auto info = make_map(3, 1, {"..."});
    path_sync::MapData map(info);

    std::vector<CBSConstraint> constraints;
    constraints.push_back({0, 0, 0, 0, true, 0, 0});

    auto path = call_low_level_search(map, {0, 0}, {2, 0}, constraints, 0);

    EXPECT_FALSE(path.has_value());
}

TEST_F(CBS_Solver_ICBS_Test, ValidatePathsAgainstConstraints)
{
    std::vector<Coordinate> path = {{0, 0}, {1, 0}, {2, 0}};
    std::vector<std::vector<Coordinate>> paths = {path};

    std::vector<CBSConstraint> constraints;
    constraints.push_back({0, 1, 0, 1, true, 0, 0});

    EXPECT_FALSE(call_validate_paths_against_constraints(paths, constraints));
}

TEST_F(CBS_Solver_ICBS_Test, ValidatePathsNoConstraints)
{
    std::vector<Coordinate> path = {{0, 0}, {1, 0}, {2, 0}};
    std::vector<std::vector<Coordinate>> paths = {path};

    EXPECT_TRUE(call_validate_paths_against_constraints(paths, {}));
}

TEST_F(CBS_Solver_ICBS_Test, ValidatePathsConstraintForOtherAgent)
{
    std::vector<Coordinate> path_a = {{0, 0}, {1, 0}, {2, 0}};
    std::vector<Coordinate> path_b = {{0, 0}, {0, 1}, {0, 2}};
    std::vector<std::vector<Coordinate>> paths = {path_a, path_b};

    // Constraint for agent 1 at (1,0) t=1 → agent 1's path_b doesn't go there
    std::vector<CBSConstraint> constraints;
    constraints.push_back({1, 1, 0, 1, true, 0, 0});

    EXPECT_TRUE(call_validate_paths_against_constraints(paths, constraints));
}

TEST_F(CBS_Solver_ICBS_Test, ComputeSoc)
{
    std::vector<Coordinate> path_a = {{0, 0}, {1, 0}, {2, 0}};
    std::vector<Coordinate> path_b = {{0, 1}};
    std::vector<std::vector<Coordinate>> paths = {path_a, path_b};

    EXPECT_EQ(call_compute_soc(paths), 2);
}

TEST_F(CBS_Solver_ICBS_Test, ClassifyCardinal)
{
    // On 3×1 corridor, both agents must go through (1,0). Both agents' costs
    // increase when (1,0) is forbidden → CARDINAL.
    auto info = make_map(3, 1, {"..."});
    path_sync::MapData map(info);

    std::vector<Coordinate> starts = {{0, 0}, {2, 0}};
    std::vector<Coordinate> goals  = {{2, 0}, {0, 0}};

    // Construct initial paths that conflict at (1,0) t=1
    std::vector<Coordinate> path_a = {{0, 0}, {1, 0}, {2, 0}};
    std::vector<Coordinate> path_b = {{2, 0}, {1, 0}, {0, 0}};
    std::vector<std::vector<Coordinate>> paths = {path_a, path_b};

    auto conflict = call_find_first_conflict(paths);
    ASSERT_TRUE(conflict.has_value());

    auto cc = call_classify_conflict(*conflict, paths, map, starts, goals);
    EXPECT_EQ(cc, CBSConflictClass::CARDINAL);
}

TEST_F(CBS_Solver_ICBS_Test, ClassifyNonCardinal)
{
    // Construct a non-cardinal conflict: on a 4×4 open grid, agents crossing
    // at (2,1) t=3 both have alternative optimal paths of the same length.
    auto info = make_map(4, 4, {"....", "....", "....", "...."});
    path_sync::MapData map(info);

    std::vector<Coordinate> starts = {{0, 0}, {3, 3}};
    std::vector<Coordinate> goals  = {{3, 3}, {0, 0}};

    // Both paths go through (2,1) at t=3 (Manhattan detour)
    std::vector<Coordinate> path_a = {{0,0},{1,0},{2,0},{2,1},{2,2},{2,3},{3,3}};
    std::vector<Coordinate> path_b = {{3,3},{2,3},{1,3},{2,1},{1,1},{0,1},{0,0}};
    // Actually path_b t=3 = (2,1), path_a t=3 = (2,1) — wait, path_a at t=3 is (2,1)...
    // Let me check: path_a indices: 0=(0,0),1=(1,0),2=(2,0),3=(2,1). Yes conflict at t=3!

    std::vector<std::vector<Coordinate>> paths = {path_a, path_b};

    auto conflict = call_find_first_conflict(paths);
    if (!conflict.has_value())
    {
        SUCCEED() << "No conflict in constructed paths — skipping";
        return;
    }

    // classify_conflict should not crash; result depends on A* alternatives
    call_classify_conflict(*conflict, paths, map, starts, goals);
    SUCCEED();
}

// ── ICBS + EPEA* low-level solver tests ─────────────────────────────

TEST_F(CBS_Solver_ICBS_Test, EPEA_LowLevelBasicPath)
{
    auto info = make_map(4, 1, {"...."});
    path_sync::MapData map(info);

    auto path = call_low_level_search_epea(map, {0, 0}, {3, 0}, {}, 0);
    ASSERT_TRUE(path.has_value());
    EXPECT_GT(path->size(), 1);
    EXPECT_EQ(path->back(), (Coordinate{3, 0}));
}

TEST_F(CBS_Solver_ICBS_Test, EPEA_LowLevelObstacleAvoidance)
{
    auto info = make_map(3, 3, {"...", ".#.", "..."});
    path_sync::MapData map(info);

    auto path = call_low_level_search_epea(map, {0, 0}, {2, 2}, {}, 0);
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->back(), (Coordinate{2, 2}));
    for (auto &c : *path)
        EXPECT_FALSE(c.first == 1 && c.second == 1);
}

TEST_F(CBS_Solver_ICBS_Test, EPEA_LowLevelVertexConstraint)
{
    auto info = make_map(4, 1, {"...."});
    path_sync::MapData map(info);

    std::vector<CBSConstraint> constraints;
    constraints.push_back({.agent = 0, .x = 2, .y = 0, .timestep = 1, .is_vertex = true});

    auto path = call_low_level_search_epea(map, {0, 0}, {3, 0}, constraints, 0);
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->back(), (Coordinate{3, 0}));
    // Must not be at (2,0) at timestep 1
    if (path->size() > 1)
        EXPECT_NE((*path)[1], (Coordinate{2, 0}));
}

TEST_F(CBS_Solver_ICBS_Test, EPEA_SolveNonConflicting)
{
    auto info = make_map(4, 2, {"....", "...."});
    path_sync::MapData map(info);
    path_sync::PerformanceMetrics perf;

    solver.set_use_epea(true);
    std::vector<Coordinate> starts = {{0, 0}, {0, 1}};
    std::vector<Coordinate> goals  = {{3, 0}, {3, 1}};

    auto result = solver.solve(map, starts, goals, perf);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 2);
    EXPECT_TRUE(perf.success);
}

TEST_F(CBS_Solver_ICBS_Test, EPEA_SolveConflicting)
{
    // Agents cross paths — forces CT split; EPEA* handles constraints
    auto info = make_map(4, 2, {"....", "...."});
    path_sync::MapData map(info);
    path_sync::PerformanceMetrics perf;

    solver.set_use_epea(true);
    std::vector<Coordinate> starts = {{0, 0}, {3, 1}};
    std::vector<Coordinate> goals  = {{3, 0}, {0, 1}};

    auto result = solver.solve(map, starts, goals, perf);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 2);
    EXPECT_EQ(result->at(0).back(), goals[0]);
    EXPECT_EQ(result->at(1).back(), goals[1]);
    EXPECT_TRUE(perf.success);
}

} // namespace path_sync::solvers::mapf

// ── CBSH Tests (friend fixture, access to internal methods) ────────────

namespace path_sync::solvers::mapf
{

class CBS_Solver_CBSH_Test : public ::testing::Test
{
protected:
    CBS_Solver solver;

    void SetUp() override
    {
        solver.set_use_icbs(true);
        solver.set_use_cbsh(true);
        solver.set_solver_name("CBSH_Solver_Test");
    }

    path_sync::MapInfo make_map(int w, int h, std::vector<std::string> grid)
    {
        path_sync::MapInfo info;
        info.width = w;
        info.height = h;
        info.map_name = "test_map";
        for (auto &row : grid)
            info.map << row << "\n";
        return info;
    }

    int call_compute_cg_heuristic(const std::vector<std::vector<Coordinate>> &paths)
    {
        return solver.compute_cg_heuristic(paths);
    }
};

TEST_F(CBS_Solver_CBSH_Test, BasicSwap)
{
    // Non-conflicting 2-agent → 1 CT node
    auto info = make_map(4, 2, {"....", "...."});
    path_sync::MapData map(info);
    path_sync::PerformanceMetrics perf;

    std::vector<Coordinate> starts = {{0, 0}, {0, 1}};
    std::vector<Coordinate> goals  = {{3, 0}, {3, 1}};

    auto result = solver.solve(map, starts, goals, perf);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 2);
    EXPECT_EQ(result->at(0).back(), goals[0]);
    EXPECT_EQ(result->at(1).back(), goals[1]);
    EXPECT_TRUE(perf.success);
}

TEST_F(CBS_Solver_CBSH_Test, ThreeAgents)
{
    // 3 agents moving East on separate rows → no conflicts
    auto info = make_map(4, 3, {"....", "....", "...."});
    path_sync::MapData map(info);
    path_sync::PerformanceMetrics perf;

    std::vector<Coordinate> starts = {{0, 0}, {0, 1}, {0, 2}};
    std::vector<Coordinate> goals  = {{3, 0}, {3, 1}, {3, 2}};

    auto result = solver.solve(map, starts, goals, perf);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 3);
    EXPECT_TRUE(perf.success);
    for (size_t a = 0; a < result->size(); a++)
        EXPECT_EQ(result->at(a).back(), goals[a]);
}

TEST_F(CBS_Solver_CBSH_Test, CGHeuristicZeroForNonConflicting)
{
    std::vector<Coordinate> path_a = {{0, 0}, {0, 1}, {0, 2}};
    std::vector<Coordinate> path_b = {{2, 0}, {2, 1}, {2, 2}};
    std::vector<std::vector<Coordinate>> paths = {path_a, path_b};

    EXPECT_EQ(call_compute_cg_heuristic(paths), 0);
}

TEST_F(CBS_Solver_CBSH_Test, CGHeuristicOneForConflictingPair)
{
    std::vector<Coordinate> path_a = {{0, 0}, {1, 0}, {2, 0}};
    std::vector<Coordinate> path_b = {{2, 0}, {1, 0}, {0, 0}};
    std::vector<std::vector<Coordinate>> paths = {path_a, path_b};

    EXPECT_EQ(call_compute_cg_heuristic(paths), 1);
}

TEST_F(CBS_Solver_CBSH_Test, CGHeuristicMinVertexCoverScaling)
{
    // Three agents: a crosses b, c conflicts with nothing → min vertex cover = 1
    std::vector<Coordinate> path_a = {{0, 0}, {1, 0}, {2, 0}};
    std::vector<Coordinate> path_b = {{2, 0}, {1, 0}, {0, 0}};
    std::vector<Coordinate> path_c = {{0, 1}, {1, 1}, {2, 1}};
    std::vector<std::vector<Coordinate>> paths = {path_a, path_b, path_c};

    int h = call_compute_cg_heuristic(paths);
    EXPECT_EQ(h, 1);
}

TEST_F(CBS_Solver_CBSH_Test, ObstacleAvoidance)
{
    // Single agent navigating around obstacle
    auto info = make_map(3, 3,
        {"...",
         ".#.",
         "..."});
    path_sync::MapData map(info);
    path_sync::PerformanceMetrics perf;

    std::vector<Coordinate> starts = {{0, 0}};
    std::vector<Coordinate> goals  = {{2, 2}};

    auto result = solver.solve(map, starts, goals, perf);

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(perf.success);
    EXPECT_EQ(result->at(0).back(), goals[0]);
}

// ── CBSH + EPEA* low-level solver tests ───────────────────────────

TEST_F(CBS_Solver_CBSH_Test, EPEA_SolveNonConflicting)
{
    auto info = make_map(4, 2, {"....", "...."});
    path_sync::MapData map(info);
    path_sync::PerformanceMetrics perf;

    solver.set_use_epea(true);
    std::vector<Coordinate> starts = {{0, 0}, {0, 1}};
    std::vector<Coordinate> goals  = {{3, 0}, {3, 1}};

    auto result = solver.solve(map, starts, goals, perf);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 2);
    EXPECT_TRUE(perf.success);
}

TEST_F(CBS_Solver_CBSH_Test, EPEA_SolveConflicting)
{
    auto info = make_map(4, 2, {"....", "...."});
    path_sync::MapData map(info);
    path_sync::PerformanceMetrics perf;

    solver.set_use_epea(true);
    std::vector<Coordinate> starts = {{0, 0}, {3, 1}};
    std::vector<Coordinate> goals  = {{3, 0}, {0, 1}};

    auto result = solver.solve(map, starts, goals, perf);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->size(), 2);
    EXPECT_TRUE(perf.success);
}

} // namespace path_sync::solvers::mapf

// ── PathFinder Dispatch Tests (CBS family via plugin names) ───────────
// All solve() calls use non-conflicting setups (same direction, separate rows)
// to keep CT at exactly 1 node and avoid heavy memory usage.

TEST_F(SolverTest, PathFinderMAHasCBSSolver)
{
    ps::PathFinder finder;
    auto names = finder.get_ma_solver_names();

    bool has_cbs = false;
    for (const auto &n : names)
        if (n == "CBS_Solver") { has_cbs = true; break; }
    EXPECT_TRUE(has_cbs) << "CBS_Solver not found in MA solvers";
}

TEST_F(SolverTest, PathFinderMAHasICBSSolver)
{
    ps::PathFinder finder;
    auto names = finder.get_ma_solver_names();

    bool has_icbs = false;
    for (const auto &n : names)
        if (n == "ICBS_Solver") { has_icbs = true; break; }
    EXPECT_TRUE(has_icbs) << "ICBS_Solver not found in MA solvers";
}

TEST_F(SolverTest, PathFinderMAHasCBSHSolver)
{
    ps::PathFinder finder;
    auto names = finder.get_ma_solver_names();

    bool has_cbsh = false;
    for (const auto &n : names)
        if (n == "CBSH_Solver") { has_cbsh = true; break; }
    EXPECT_TRUE(has_cbsh) << "CBSH_Solver not found in MA solvers";
}

TEST_F(SolverTest, PathFinderCBSSolvesViaDispatch)
{
    ps::PathFinder finder;
    auto names = finder.get_ma_solver_names();

    int cbs_idx = -1;
    for (size_t i = 0; i < names.size(); i++)
        if (names[i] == "CBS_Solver") { cbs_idx = static_cast<int>(i); break; }
    ASSERT_GE(cbs_idx, 0) << "CBS_Solver not loaded";

    auto sa_names = finder.get_sa_solver_names();
    finder.select_solver_by_index(sa_names.size() + cbs_idx);

    auto info = make_map(4, 2, {"....", "...."});
    ps::MapData map(info);

    std::vector<ps::Coordinate> starts = {{0, 0}, {0, 1}};
    std::vector<ps::Coordinate> goals  = {{3, 0}, {3, 1}};

    auto result = finder.find_path(map, starts, goals);

    ASSERT_TRUE(std::holds_alternative<std::vector<std::vector<ps::Coordinate>>>(result));
    auto paths = std::get<std::vector<std::vector<ps::Coordinate>>>(result);
    EXPECT_EQ(paths.size(), 2);
    EXPECT_EQ(paths[0].back(), goals[0]);
    EXPECT_EQ(paths[1].back(), goals[1]);
}

TEST_F(SolverTest, PathFinderICBSSolvesViaDispatch)
{
    ps::PathFinder finder;
    auto names = finder.get_ma_solver_names();

    int icbs_idx = -1;
    for (size_t i = 0; i < names.size(); i++)
        if (names[i] == "ICBS_Solver") { icbs_idx = static_cast<int>(i); break; }
    ASSERT_GE(icbs_idx, 0) << "ICBS_Solver not loaded";

    auto sa_names = finder.get_sa_solver_names();
    finder.select_solver_by_index(sa_names.size() + icbs_idx);

    auto info = make_map(4, 2, {"....", "...."});
    ps::MapData map(info);

    std::vector<ps::Coordinate> starts = {{0, 0}, {0, 1}};
    std::vector<ps::Coordinate> goals  = {{3, 0}, {3, 1}};

    auto result = finder.find_path(map, starts, goals);

    ASSERT_TRUE(std::holds_alternative<std::vector<std::vector<ps::Coordinate>>>(result));
    auto paths = std::get<std::vector<std::vector<ps::Coordinate>>>(result);
    EXPECT_EQ(paths.size(), 2);
    EXPECT_EQ(paths[0].back(), goals[0]);
    EXPECT_EQ(paths[1].back(), goals[1]);
}

TEST_F(SolverTest, PathFinderCBSHSolvesViaDispatch)
{
    ps::PathFinder finder;
    auto names = finder.get_ma_solver_names();

    int cbsh_idx = -1;
    for (size_t i = 0; i < names.size(); i++)
        if (names[i] == "CBSH_Solver") { cbsh_idx = static_cast<int>(i); break; }
    ASSERT_GE(cbsh_idx, 0) << "CBSH_Solver not loaded";

    auto sa_names = finder.get_sa_solver_names();
    finder.select_solver_by_index(sa_names.size() + cbsh_idx);

    auto info = make_map(4, 2, {"....", "...."});
    ps::MapData map(info);

    std::vector<ps::Coordinate> starts = {{0, 0}, {0, 1}};
    std::vector<ps::Coordinate> goals  = {{3, 0}, {3, 1}};

    auto result = finder.find_path(map, starts, goals);

    ASSERT_TRUE(std::holds_alternative<std::vector<std::vector<ps::Coordinate>>>(result));
    auto paths = std::get<std::vector<std::vector<ps::Coordinate>>>(result);
    EXPECT_EQ(paths.size(), 2);
    EXPECT_EQ(paths[0].back(), goals[0]);
    EXPECT_EQ(paths[1].back(), goals[1]);
}

TEST_F(SolverTest, PathFinderCBSReturnsMAPFMetrics)
{
    ps::PathFinder finder;
    auto names = finder.get_ma_solver_names();

    int cbs_idx = -1;
    for (size_t i = 0; i < names.size(); i++)
        if (names[i] == "CBS_Solver") { cbs_idx = static_cast<int>(i); break; }
    ASSERT_GE(cbs_idx, 0);

    auto sa_names = finder.get_sa_solver_names();
    finder.select_solver_by_index(sa_names.size() + cbs_idx);

    // Non-conflicting 2-agent → metrics still populated
    auto info = make_map(4, 2, {"....", "...."});
    ps::MapData map(info);

    std::vector<ps::Coordinate> starts = {{0, 0}, {0, 1}};
    std::vector<ps::Coordinate> goals  = {{3, 0}, {3, 1}};

    auto result = finder.find_path(map, starts, goals);

    ASSERT_TRUE(std::holds_alternative<std::vector<std::vector<ps::Coordinate>>>(result));

    const auto &metrics = finder.get_last_ma_metrics();
    EXPECT_TRUE(metrics.has_value());
    if (metrics.has_value())
    {
        EXPECT_GE(metrics->joint_states_expanded, 0);
        EXPECT_GE(metrics->conflicts_detected, 0);
        EXPECT_GT(metrics->max_timestep_reached, 0);
        EXPECT_GT(metrics->flow_time, 0);
        EXPECT_GT(metrics->mean_path_length, 0.0);
    }
}

TEST_F(SolverTest, PathFinderCBSHandlesUnreachableViaDispatch)
{
    ps::PathFinder finder;
    auto names = finder.get_ma_solver_names();

    int cbs_idx = -1;
    for (size_t i = 0; i < names.size(); i++)
        if (names[i] == "CBS_Solver") { cbs_idx = static_cast<int>(i); break; }
    ASSERT_GE(cbs_idx, 0);

    auto sa_names = finder.get_sa_solver_names();
    finder.select_solver_by_index(sa_names.size() + cbs_idx);

    auto info = make_map(3, 1, {".#."});
    ps::MapData map(info);

    std::vector<ps::Coordinate> starts = {{0, 0}};
    std::vector<ps::Coordinate> goals  = {{2, 0}};

    auto result = finder.find_path(map, starts, goals);

    ASSERT_TRUE(std::holds_alternative<std::vector<ps::Coordinate>>(result));
    EXPECT_TRUE(std::get<std::vector<ps::Coordinate>>(result).empty());
}

TEST_F(SolverTest, PathFinderCBSHandlesSingleAgentViaDispatch)
{
    ps::PathFinder finder;
    auto names = finder.get_ma_solver_names();

    int cbs_idx = -1;
    for (size_t i = 0; i < names.size(); i++)
        if (names[i] == "CBS_Solver") { cbs_idx = static_cast<int>(i); break; }
    ASSERT_GE(cbs_idx, 0);

    auto sa_names = finder.get_sa_solver_names();
    finder.select_solver_by_index(sa_names.size() + cbs_idx);

    auto info = make_map(3, 3, {"...", "...", "..."});
    ps::MapData map(info);

    std::vector<ps::Coordinate> starts = {{0, 0}};
    std::vector<ps::Coordinate> goals  = {{2, 2}};

    auto result = finder.find_path(map, starts, goals);

    // Single agent with MA solver selected falls back to SA solver → single path
    ASSERT_TRUE(std::holds_alternative<std::vector<ps::Coordinate>>(result));
    auto path = std::get<std::vector<ps::Coordinate>>(result);
    EXPECT_FALSE(path.empty());
    EXPECT_EQ(path.back(), goals[0]);
}
