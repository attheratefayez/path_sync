#include <gtest/gtest.h>
#include <cstring>
#include "path_sync_core/performance_mat.hpp"

using namespace path_sync;

TEST(PerformanceMetricsTest, SuboptimalityRatioWithOptimalZero)
{
    PerformanceMetrics pm;
    pm.path_length = 10;
    pm.optimal_path_length = 0;
    EXPECT_FLOAT_EQ(pm.suboptimality_ratio(), 1.0f);
}

TEST(PerformanceMetricsTest, SuboptimalityRatioCorrect)
{
    PerformanceMetrics pm;
    pm.path_length = 15;
    pm.optimal_path_length = 10;
    EXPECT_FLOAT_EQ(pm.suboptimality_ratio(), 1.5f);
}

TEST(PerformanceMetricsTest, SuboptimalityRatioOptimalPath)
{
    PerformanceMetrics pm;
    pm.path_length = 10;
    pm.optimal_path_length = 10;
    EXPECT_FLOAT_EQ(pm.suboptimality_ratio(), 1.0f);
}

TEST(PerformanceMetricsTest, CsvHeaderMatchesExpected)
{
    std::string hdr = PerformanceMetrics::csv_header();
    EXPECT_EQ(hdr, "solver_name,map_name,scene_id,num_agents,success,runtime_us,"
                   "path_length,optimal_path_length,"
                   "suboptimality_ratio,nodes_explored,nodes_expanded,nodes_reopened,peak_open_size,"
                   "sum_of_costs,makespan,timestamp");
}

TEST(PerformanceMetricsTest, CsvLineFormat)
{
    PerformanceMetrics pm;
    pm.solver_name = "Astar_Solver";
    pm.map_name = "test.map";
    pm.scene_id = 5;
    pm.num_agents = 2;
    pm.success = true;
    pm.runtime = std::chrono::microseconds(1234);
    pm.path_length = 42;
    pm.optimal_path_length = 40;
    pm.num_of_nodes_explored = 100;
    pm.num_of_nodes_expanded = 50;
    pm.num_of_nodes_reopened = 2;
    pm.peak_open_size = 25;
    pm.sum_of_costs = 80;
    pm.makespan = 15;

    std::string line = pm.csv_line();
    // timestamp varies, so we check the prefix
    EXPECT_TRUE(line.find("Astar_Solver,test.map,5,2,1,1234,42,40,") == 0)
        << "Actual line: " << line;
    EXPECT_TRUE(line.find(",80,15,") != std::string::npos)
        << "Missing MAPF fields, line: " << line;
}

TEST(PerformanceMetricsTest, CsvLineFailedSolve)
{
    PerformanceMetrics pm;
    pm.solver_name = "BFS_Solver";
    pm.map_name = "fail.map";
    pm.scene_id = 0;
    pm.num_agents = 1;
    pm.success = false;
    pm.runtime = std::chrono::microseconds(0);

    std::string line = pm.csv_line();
    EXPECT_TRUE(line.find("BFS_Solver,fail.map,0,1,0,0,0,0,") == 0)
        << "Actual line: " << line;
}

TEST(PerformanceMetricsTest, ReportContainsKeyFields)
{
    PerformanceMetrics pm;
    pm.solver_name = "JPS_Solver";
    pm.map_name = "test.map";
    pm.success = true;
    pm.runtime = std::chrono::microseconds(555);
    pm.path_length = 10;
    pm.num_of_nodes_explored = 99;
    pm.num_of_nodes_expanded = 44;
    pm.num_of_nodes_reopened = 1;
    pm.peak_open_size = 20;

    std::string r = pm.report().str();
    EXPECT_TRUE(r.find("JPS_Solver") != std::string::npos);
    EXPECT_TRUE(r.find("OK") != std::string::npos);
    EXPECT_TRUE(r.find("555us") != std::string::npos);
    EXPECT_TRUE(r.find("explored: 99") != std::string::npos);
}

TEST(PerformanceMetricsTest, ReportIncludesMAPFWhenSet)
{
    PerformanceMetrics pm;
    pm.sum_of_costs = 80;
    pm.makespan = 15;

    std::string r = pm.report().str();
    EXPECT_TRUE(r.find("soc: 80") != std::string::npos);
    EXPECT_TRUE(r.find("makespan: 15") != std::string::npos);
}

TEST(PerformanceMetricsTest, ReportOmitsMAPFWhenZero)
{
    PerformanceMetrics pm;
    pm.sum_of_costs = 0;
    pm.makespan = 0;

    std::string r = pm.report().str();
    EXPECT_TRUE(r.find("soc:") == std::string::npos);
}

TEST(PerformanceMetricsTest, FmtTimestampZero)
{
    EXPECT_EQ(PerformanceMetrics::fmt_timestamp(0), "");
}

TEST(PerformanceMetricsTest, FmtTimestampNonZero)
{
    std::time_t t = std::time(nullptr);
    std::string result = PerformanceMetrics::fmt_timestamp(t);
    EXPECT_FALSE(result.empty());
    EXPECT_EQ(result.size(), 19);
    EXPECT_EQ(result[4], '-');
    EXPECT_EQ(result[7], '-');
    EXPECT_EQ(result[10], ' ');
    EXPECT_EQ(result[13], ':');
    EXPECT_EQ(result[16], ':');
}
