#ifndef __PATH_SYNC_PERFORMANCE_MET_HPP__
#define __PATH_SYNC_PERFORMANCE_MET_HPP__

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

namespace path_sync
{

struct PerformanceMetrics
{
    // Identification
    std::string solver_name;
    std::string map_name;
    int scene_id = -1;
    int num_agents = 0;
    std::time_t timestamp = 0;

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

    // Timeout support
    std::chrono::steady_clock::time_point deadline;
    bool has_timeout = false;

    bool timed_out() const noexcept
    {
        return has_timeout && std::chrono::steady_clock::now() >= deadline;
    }

    // MAPF-specific
    std::size_t sum_of_costs = 0;
    std::size_t makespan = 0;

    // Extended MAPF metrics (populated by multi-agent solvers)
    struct MAPFMetrics *mapf_metrics = nullptr;

    float suboptimality_ratio() const noexcept
    {
        if (optimal_path_length == 0) return 1.0f;
        return static_cast<float>(path_length) / static_cast<float>(optimal_path_length);
    }

    std::stringstream report() const
    {
        std::stringstream ss;
        ss << solver_name << " " << map_name << " "
           << (success ? "OK" : "FAIL") << " "
           << "agents: " << num_agents << " "
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
        return "solver_name,map_name,scene_id,num_agents,success,runtime_us,path_length,optimal_path_length,"
               "suboptimality_ratio,nodes_explored,nodes_expanded,nodes_reopened,peak_open_size,"
               "sum_of_costs,makespan,timestamp";
    }

    static std::string fmt_timestamp(std::time_t t)
    {
        if (t == 0) return "";
        std::tm tm{};
        localtime_r(&t, &tm);
        char buf[32];
        std::strftime(buf, sizeof buf, "%Y-%m-%d %H:%M:%S", &tm);
        return buf;
    }

    std::string csv_line() const
    {
        std::stringstream ss;
        ss << solver_name << ","
           << map_name << ","
           << scene_id << ","
           << num_agents << ","
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
           << makespan << ","
           << fmt_timestamp(timestamp);
        return ss.str();
    }
};

// ── Failure reason for multi-agent solvers ───────────────────────────

enum class MAFailureReason
{
    NONE,
    PATH_NOT_FOUND,
    CT_EXHAUSTED,
    CANCELLED,
    TIMEOUT
};

// ── Extended metrics for multi-agent solves ──────────────────────────

struct MAPFMetrics
{
    // Conflicts
    std::size_t conflicts_detected = 0;
    std::size_t conflicts_resolved = 0;

    // Joint search
    std::size_t joint_states_expanded = 0;
    std::size_t joint_states_explored = 0;

    // Replanning (MStar specific)
    std::size_t num_replan_iterations = 0;
    std::size_t num_joint_searches = 0;
    std::size_t num_joint_search_failures = 0;
    std::size_t final_joint_set_size = 0;

    // Depth
    std::size_t solution_depth = 0;
    std::size_t max_timestep_reached = 0;

    // Failure reason (populated by multi-agent solvers on failure)
    MAFailureReason failure_reason = MAFailureReason::NONE;

    // Unresolved conflict details (when failure_reason == CT_EXHAUSTED)
    int last_conflict_agent_a = -1;
    int last_conflict_agent_b = -1;
    int last_conflict_x = -1;
    int last_conflict_y = -1;
    int last_conflict_timestep = -1;

    // Per-agent aggregates (computed from solution paths)
    std::vector<std::size_t> individual_path_lengths;
    std::size_t flow_time = 0;
    double mean_path_length = 0.0;
    std::size_t min_path_length = 0;
    std::size_t max_path_length = 0;

    void compute_aggregates()
    {
        if (individual_path_lengths.empty())
            return;
        flow_time = std::accumulate(individual_path_lengths.begin(), individual_path_lengths.end(), std::size_t{0});
        min_path_length = *std::min_element(individual_path_lengths.begin(), individual_path_lengths.end());
        max_path_length = *std::max_element(individual_path_lengths.begin(), individual_path_lengths.end());
        mean_path_length = static_cast<double>(flow_time) / individual_path_lengths.size();
    }

    static std::string csv_header()
    {
        return "ma_solver_name,map_name,scene_id,num_agents,success,runtime_us,"
               "sum_of_costs,makespan,flow_time,"
               "mean_path_len,min_path_len,max_path_len,"
               "joint_states_expanded,joint_states_explored,"
               "conflicts_detected,conflicts_resolved,"
               "replan_iterations,joint_searches,joint_search_failures,"
               "final_joint_set_size,solution_depth,max_timestep,"
               "failure_reason,last_conflict_a,last_conflict_b,"
               "last_conflict_x,last_conflict_y,last_conflict_t,timestamp";
    }

    std::string csv_line(const PerformanceMetrics &base) const
    {
        static const char *reason_names[] = {"NONE","PATH_NOT_FOUND","CT_EXHAUSTED","CANCELLED","TIMEOUT"};
        int r_idx = static_cast<int>(failure_reason);
        if (r_idx < 0 || r_idx > 4) r_idx = 0;

        std::stringstream ss;
        ss << base.solver_name << ","
           << base.map_name << ","
           << base.scene_id << ","
           << base.num_agents << ","
           << (base.success ? 1 : 0) << ","
           << base.runtime.count() << ","
           << base.sum_of_costs << ","
           << base.makespan << ","
           << flow_time << ","
           << mean_path_length << ","
           << min_path_length << ","
           << max_path_length << ","
           << joint_states_expanded << ","
           << joint_states_explored << ","
           << conflicts_detected << ","
           << conflicts_resolved << ","
           << num_replan_iterations << ","
           << num_joint_searches << ","
           << num_joint_search_failures << ","
           << final_joint_set_size << ","
           << solution_depth << ","
           << max_timestep_reached << ","
           << reason_names[r_idx] << ","
           << last_conflict_agent_a << ","
           << last_conflict_agent_b << ","
           << last_conflict_x << ","
           << last_conflict_y << ","
           << last_conflict_timestep << ","
           << PerformanceMetrics::fmt_timestamp(base.timestamp);
        return ss.str();
    }
};

} // namespace path_sync
#endif // !__PATH_SYNC_PERFORMANCE_MET_HPP__
