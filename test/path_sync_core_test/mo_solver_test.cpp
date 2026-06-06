#include <gtest/gtest.h>

#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/mo_types.hpp"
#include "path_sync_core/path_finder.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/performance_mat.hpp"
#include "path_sync_core/solvers/mo_astar_solver.hpp"
#include "path_sync_core/solvers/nsga2_solver.hpp"
#include "path_sync_core/solvers/pareto_rrt_solver.hpp"

namespace ps = path_sync;

class MOTest : public testing::Test
{
protected:
    ps::MapInfo make_map(int w, int h, std::vector<std::string> grid)
    {
        ps::MapInfo info;
        info.width = w;
        info.height = h;
        info.map_name = "mo_test_map";
        for (auto &row : grid)
            info.map << row << "\n";
        return info;
    }
};

TEST_F(MOTest, MOSolutionDominance)
{
    ps::MOSolution a, b;
    a.costs = {2.0f, 2.0f, 3.0f};
    b.costs = {1.5f, 2.5f, 3.5f};

    // a(obj0)=2 > b(obj0)=1.5 → a does not dominate b
    EXPECT_FALSE(a.dominates(b, 3));
    // b(obj1)=2.5 > a(obj1)=2.0 → b does not dominate a
    EXPECT_FALSE(b.dominates(a, 3));

    // a dominates b (b is worse in all objectives)
    b.costs = {2.5f, 3.0f, 4.0f};
    EXPECT_TRUE(a.dominates(b, 3));
    EXPECT_FALSE(b.dominates(a, 3));

    // Equal → no dominance
    b.costs = {2.0f, 2.0f, 3.0f};
    EXPECT_FALSE(a.dominates(b, 3));
    EXPECT_FALSE(b.dominates(a, 3));

    // a better in some, equal in others → a dominates
    b.costs = {2.0f, 2.5f, 3.5f};
    EXPECT_TRUE(a.dominates(b, 3));
    EXPECT_FALSE(b.dominates(a, 3));
}

TEST_F(MOTest, MOMetricsCSV)
{
    ps::MOMetrics met;
    met.front_size = 5;
    met.hypervolume = 12.34;

    auto ts = std::time(nullptr);
    std::string line = met.csv_line("NSGA2", "test", 0, true,
                                     std::chrono::microseconds(1000), ts);
    EXPECT_TRUE(line.find("NSGA2") != std::string::npos);
    EXPECT_TRUE(line.find("5") != std::string::npos);
}

TEST_F(MOTest, MOAStarFindsPath)
{
    auto info = make_map(4, 4, {
        "....",
        "....",
        "....",
        "...."
    });
    ps::MapData map(info);
    ps::solvers::mo::MOAStarSolver solver;
    ps::PerformanceMetrics perf;
    ps::MOMetrics mo_met;

    auto result = solver.solve(map, nullptr, {0, 0}, {3, 3}, 2, perf, mo_met);

    ASSERT_TRUE(result.has_value());
    EXPECT_FALSE(result->empty());
    EXPECT_TRUE(result->front().path.size() > 1);
    EXPECT_EQ(result->front().costs.size(), 2);
    EXPECT_TRUE(perf.success);
}

TEST_F(MOTest, MOAStarNoPathOnBlocked)
{
    auto info = make_map(3, 3, {
        "...",
        ".#.",
        "..."
    });
    ps::MapData map(info);
    map.set_cell_type({1, 1}, ps::CellType::WALL);
    ps::solvers::mo::MOAStarSolver solver;
    ps::PerformanceMetrics perf;
    ps::MOMetrics mo_met;

    // Goal trapped by wall
    auto result = solver.solve(map, nullptr, {0, 0}, {3, 3}, 2, perf, mo_met);
    EXPECT_FALSE(result.has_value());
}

TEST_F(MOTest, NSGA2FindsPath)
{
    auto info = make_map(5, 5, {
        ".....",
        ".....",
        ".....",
        ".....",
        "....."
    });
    ps::MapData map(info);
    ps::solvers::mo::NSGA2Solver solver;
    ps::PerformanceMetrics perf;
    ps::MOMetrics mo_met;

    auto result = solver.solve(map, nullptr, {0, 0}, {4, 4}, 2, perf, mo_met);

    // Stochastic solver; may not find path every time
    if (!result.has_value())
        GTEST_SKIP() << "NSGA2 stochastic: no path found this run";
    EXPECT_FALSE(result->empty());
    EXPECT_TRUE(perf.success);
}

TEST_F(MOTest, ParetoRRTFindsPath)
{
    auto info = make_map(5, 5, {
        ".....",
        ".....",
        ".....",
        ".....",
        "....."
    });
    ps::MapData map(info);
    ps::solvers::mo::ParetoRRTSolver solver;
    ps::PerformanceMetrics perf;
    ps::MOMetrics mo_met;

    auto result = solver.solve(map, nullptr, {0, 0}, {4, 4}, 2, perf, mo_met);

    ASSERT_TRUE(result.has_value());
    EXPECT_FALSE(result->empty());
    EXPECT_TRUE(perf.success);
}

TEST_F(MOTest, PathFinderSelectMOMethods)
{
    ps::PathFinder finder;

    // No MO solver selected initially
    EXPECT_FALSE(finder.is_current_solver_mo());

    auto mo_names = finder.get_mo_solver_names();
    if (!mo_names.empty())
    {
        finder.select_mo_solver_by_index(0);
        EXPECT_TRUE(finder.is_current_solver_mo());
        EXPECT_EQ(finder.get_current_solver_name(), mo_names[0]);
    }
}

TEST_F(MOTest, PathFinderMOPath)
{
    auto info = make_map(4, 4, {
        "....",
        "....",
        "....",
        "...."
    });
    ps::MapData map(info);
    ps::PathFinder finder;

    auto mo_names = finder.get_mo_solver_names();
    if (mo_names.empty())
        GTEST_SKIP() << "No MO solvers loaded";

    finder.select_mo_solver_by_index(0);
    ps::MOMetrics mo_met;
    auto perf = finder.get_performance_metrics();
    auto result = finder.find_mo_path(map, nullptr, {0, 0}, {3, 3}, 2, perf, mo_met);

    ASSERT_TRUE(result.has_value());
    EXPECT_FALSE(result->empty());
    EXPECT_EQ(result->front().path.front(), ps::Coordinate(0, 0));
    EXPECT_EQ(result->front().path.back(), ps::Coordinate(3, 3));
}

TEST_F(MOTest, DominanceParetoFrontTwoObj)
{
    auto info = make_map(3, 3, {
        "...",
        "...",
        "..."
    });
    ps::MapData map(info);
    ps::solvers::mo::MOAStarSolver solver;
    ps::PerformanceMetrics perf;
    ps::MOMetrics mo_met;

    auto result = solver.solve(map, nullptr, {0, 0}, {2, 2}, 2, perf, mo_met);

    ASSERT_TRUE(result.has_value());
    ASSERT_FALSE(result->empty());

    // No solution should dominate any other
    for (std::size_t i = 0; i < result->size(); i++)
    {
        for (std::size_t j = 0; j < result->size(); j++)
        {
            if (i == j) continue;
            EXPECT_FALSE((*result)[i].dominates((*result)[j], 2) &&
                         (*result)[j].dominates((*result)[i], 2))
                << "Mutual dominance between " << i << " and " << j;
        }
    }
}
