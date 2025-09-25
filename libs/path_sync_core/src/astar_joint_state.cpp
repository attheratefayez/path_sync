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
    // NOTE: can be set to raise a error too if path_finder can handle and process error
    if (starts.size() != goals.size())
        return std::nullopt;

    // INITIALIZATION START
    mapf_type::JointState new_state;
    new_state.positions = starts;
    new_state.time = 0;

    mapf_type::NodePtr parent_node =
        std::make_shared<mapf_type::Node>(new_state, 0, util_funcs::heuristic(starts, goals));

    std::priority_queue<mapf_type::NodePtr, std::vector<mapf_type::NodePtr>, mapf_type::CompareGreaterNode> open_set;
    open_set.push(parent_node);

    std::map<mapf_type::JointState, int> closed_set;
    // INITIALIZATION END

    while (!open_set.empty())
    {
        mapf_type::NodePtr current = open_set.top();
        open_set.pop();

        if (current->_state.positions == goals)
            return util_funcs::extract_paths(current);

        if (closed_set.contains(current->_state) && closed_set[current->_state] <= current->_g_score)
            continue;

        closed_set[current->_state] = current->_g_score;

        std::optional<std::vector<std::vector<Coordinate>>> possible_actions =
            util_funcs::possible_actions_with_state(current->_state, map_data);

        if (!possible_actions.has_value())
            return std::nullopt;

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
        }
    }

    return std::nullopt;
}
