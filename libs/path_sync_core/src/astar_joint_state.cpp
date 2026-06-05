#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>

#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/solvers/astar_joint_state.hpp"
#include "path_sync_core/solvers/astar_joint_state_utils.hpp"

using mapf_astar = path_sync::solvers::mapf::Astar_Joint_State_Solver;
using util_funcs = mapf::astar_joint_state::Utils;

std::string_view mapf_astar::get_solver_name() const
{
    return solver_name_;
}

std::optional<std::vector<std::vector<path_sync::Coordinate>>> mapf_astar::solve(const path_sync::MapData &map_data, std::vector<Coordinate> starts, std::vector<Coordinate> goals,
                                     path_sync::PerformanceMetrics &performance_met)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    if (starts.size() != goals.size())
        return std::nullopt;

    mapf_type::JointState new_state;
    new_state.positions = starts;
    new_state.time = 0;

    mapf_type::NodePtr parent_node =
        std::make_shared<mapf_type::Node>(new_state, 0, util_funcs::heuristic(starts, goals));

    std::priority_queue<mapf_type::NodePtr, std::vector<mapf_type::NodePtr>, mapf_type::CompareGreaterNode> open_set;
    open_set.push(parent_node);
    performance_met.peak_open_size = open_set.size();

    std::map<mapf_type::JointState, int> closed_set;

    while (!open_set.empty())
    {
        mapf_type::NodePtr current = open_set.top();
        open_set.pop();
        if (performance_met.cancel_flag && *performance_met.cancel_flag) break;
        performance_met.num_of_nodes_expanded++;

        if (current->_state.positions == goals)
        {
            auto end_time = std::chrono::high_resolution_clock::now();
            performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            performance_met.success = true;
            return util_funcs::extract_paths(current);
        }

        if (closed_set.contains(current->_state) && closed_set[current->_state] <= current->_g_score)
            continue;

        closed_set[current->_state] = current->_g_score;

        std::optional<std::vector<std::vector<Coordinate>>> possible_actions =
            util_funcs::possible_actions_with_state(current->_state, map_data);

        if (!possible_actions.has_value())
        {
            auto end_time = std::chrono::high_resolution_clock::now();
            performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            return std::nullopt;
        }

        for (std::vector<Coordinate> &action : possible_actions.value())
        {
            mapf_type::JointState new_state = util_funcs::apply_actions(current->_state, action);

            if (not util_funcs::check_validity_of_state(current->_state, new_state))
                continue;

            std::size_t g_score = current->_g_score + 1;
            if (closed_set.contains(new_state) and closed_set[new_state] <= g_score)
                continue;

            std::size_t h_score = util_funcs::heuristic(new_state.positions, goals);
            mapf_type::NodePtr new_node = std::make_shared<mapf_type::Node>(new_state, g_score, h_score, current);
            open_set.push(new_node);
            if (open_set.size() > performance_met.peak_open_size)
                performance_met.peak_open_size = open_set.size();
            performance_met.num_of_nodes_explored++;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    return std::nullopt;
}

#ifdef PATH_SYNC_BUILD_AS_PLUGIN
extern "C"
{

const char *plugin_name() { return "Astar_Joint_State_Solver"; }
bool plugin_is_optimal() { return true; }
bool plugin_is_multi_agent() { return true; }
void *plugin_create() { return new path_sync::solvers::mapf::Astar_Joint_State_Solver(); }
void plugin_destroy(void *p) { delete static_cast<path_sync::solvers::mapf::Astar_Joint_State_Solver *>(p); }

}
#endif
