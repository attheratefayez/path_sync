#ifndef __PATH_SYNC_PERFORMANCE_MET_HPP__
#define __PATH_SYNC_PERFORMANCE_MET_HPP__

#include <atomic>
#include <chrono>
#include <cstdint>
#include <sstream>
#include <string>

namespace path_sync
{

struct PerformanceMetrics
{
    // Identification
    std::string solver_name;
    std::string map_name;
    int scene_id = -1;

    // Success
    bool success = false;

    // Timing
    std::chrono::microseconds runtime{0};

    // Path quality
    std::size_t path_length = 0;
    std::size_t optimal_path_length = 0; // filled externally for optimality ratio

    // Search effort
    std::size_t num_of_nodes_explored = 0;
    std::size_t num_of_nodes_expanded = 0;
    std::size_t num_of_nodes_reopened = 0;
    std::size_t peak_open_size = 0;

    // Cancel support (non-owning pointer)
    std::atomic<bool> *cancel_flag = nullptr;

    // MAPF-specific
    std::size_t sum_of_costs = 0;
    std::size_t makespan = 0;

    float suboptimality_ratio() const
    {
        if (optimal_path_length == 0) return 1.0f;
        return static_cast<float>(path_length) / static_cast<float>(optimal_path_length);
    }

    std::stringstream report() const
    {
        std::stringstream ss;
        ss << solver_name << " " << map_name << " "
           << (success ? "OK" : "FAIL") << " "
           << "rt: " << runtime.count() << "us "
           << "len: " << path_length << " "
           << "explored: " << num_of_nodes_explored << " "
           << "expanded: " << num_of_nodes_expanded << " "
           << "reopened: " << num_of_nodes_reopened << " "
           << "peak_open: " << peak_open_size;
        if (sum_of_costs > 0)
            ss << " soc: " << sum_of_costs << " makespan: " << makespan;
        ss << std::endl;
        return ss;
    }

    static std::string csv_header()
    {
        return "solver_name,map_name,scene_id,success,runtime_us,path_length,optimal_path_length,"
               "suboptimality_ratio,nodes_explored,nodes_expanded,nodes_reopened,peak_open_size,"
               "sum_of_costs,makespan";
    }

    std::string csv_line() const
    {
        std::stringstream ss;
        ss << solver_name << ","
           << map_name << ","
           << scene_id << ","
           << (success ? 1 : 0) << ","
           << runtime.count() << ","
           << path_length << ","
           << optimal_path_length << ","
           << suboptimality_ratio() << ","
           << num_of_nodes_explored << ","
           << num_of_nodes_expanded << ","
           << num_of_nodes_reopened << ","
           << peak_open_size << ","
           << sum_of_costs << ","
           << makespan;
        return ss.str();
    }
};

} // namespace path_sync
#endif // !__PATH_SYNC_PERFORMANCE_MET_HPP__
