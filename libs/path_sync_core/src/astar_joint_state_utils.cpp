#include <algorithm>
#include <set>

#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/solvers/astar_joint_state_utils.hpp"

namespace mapf
{
namespace astar_joint_state
{

float Utils::manhattan_distance(path_sync::Coordinate pos1, path_sync::Coordinate pos2)
{
    return std::abs(pos1.first - pos2.first) + std::abs(pos1.second - pos2.second);
}

float Utils::heuristic(std::vector<path_sync::Coordinate> starts, std::vector<path_sync::Coordinate> goals)
{
    float h_score = 0;

    for (std::size_t idx = 0; idx < starts.size(); ++idx)
    {
        h_score += Utils::manhattan_distance(starts[idx], goals[idx]);
    }

    return h_score;
}

std::optional<std::vector<std::vector<path_sync::Coordinate>>> Utils::cartesian_product(
    std::vector<std::vector<path_sync::Coordinate>> input_vec)
{
    if (input_vec.size() < 2)
        return std::nullopt;

    std::vector<std::vector<path_sync::Coordinate>> res;

    for (auto n : input_vec[0])
    {
        res.emplace_back(1, n);
    }

    for (int idx = 1; idx < input_vec.size(); ++idx)
    {
        Utils::cartesian_product_underlying(res, input_vec[idx]);
    }

    return res;
}

void Utils::cartesian_product_underlying(std::vector<std::vector<path_sync::Coordinate>> &res,
                                         const std::vector<path_sync::Coordinate> &in1)
{
    std::vector<std::vector<path_sync::Coordinate>> result;

    for (const path_sync::Coordinate &n : in1)
    {
        for (std::vector<path_sync::Coordinate> part_res : res)
        {
            part_res.push_back(n);
            result.push_back(part_res);
        }
    }

    res = result;
}

std::optional<std::vector<std::vector<path_sync::Coordinate>>> Utils::possible_actions_with_state(
    const path_sync::mapf_type::JointState &state, const path_sync::MapData &map_data)
{

    std::vector<std::vector<path_sync::Coordinate>> filtered_moves;

    for (const path_sync::Coordinate &agent : state.positions)
    {
        filtered_moves.push_back(Utils::possible_actions_with_agent(agent, map_data));
    }

    std::optional<std::vector<std::vector<path_sync::Coordinate>>> ret_val = Utils::cartesian_product(filtered_moves);

    return ret_val;
}

std::vector<path_sync::Coordinate> Utils::possible_actions_with_agent(const path_sync::Coordinate &agent,
                                                                      const path_sync::MapData &map_data)
{
    std::vector<path_sync::Coordinate> moves = {path_sync::Coordinate(0, 0), path_sync::Coordinate(1, 0),
                                                path_sync::Coordinate(0, -1), path_sync::Coordinate(-1, 0),
                                                path_sync::Coordinate(0, 1)};

    std::vector<path_sync::Coordinate> agent_moves;

    for (const path_sync::Coordinate &move : moves)
    {
        int x = agent.first + move.first;
        int y = agent.second + move.second;

        if (map_data.get_cell_type(x, y) != path_sync::CellType::WALL)
        {
            agent_moves.push_back(path_sync::Coordinate(x, y));
        }
    }

    return agent_moves;
}

path_sync::mapf_type::JointState Utils::apply_actions(const path_sync::mapf_type::JointState &state,
                                           const std::vector<path_sync::Coordinate> &actions)
{
    if (state.positions.size() != actions.size())
        throw std::logic_error("[_apply_action] states actions size should be same.");

    path_sync::mapf_type::JointState new_state;

    for (int agent = 0; agent > state.positions.size(); ++agent)
    {
        new_state.positions.push_back(Utils::apply_single_action(state.positions[agent], actions[agent]));
    }

    return new_state;
}

path_sync::Coordinate Utils::apply_single_action(path_sync::Coordinate agent, path_sync::Coordinate action)
{
    path_sync::Coordinate new_pos;
    new_pos.first = agent.first + action.first;
    new_pos.second = agent.second + action.second;

    return new_pos;
}

bool Utils::check_validity_of_state(const path_sync::mapf_type::JointState &current_state, const path_sync::mapf_type::JointState &new_state)
{
    // TODO: check if there is any vertex conflict, then check if there is any edge conflict

    // check for vertex-conflict
    if (std::set(new_state.positions.begin(), new_state.positions.end()).size() < new_state.positions.size())
        return false;

    // check for edge-conflict
    for (int i = 0; i < new_state.positions.size(); ++i)
        for (int j = i + 1; j < new_state.positions.size(); ++j)
        {
            if (new_state.positions[i] == current_state.positions[j] and
                new_state.positions[j] == current_state.positions[i])
                return false;
        }

    return true;
}

std::vector<std::vector<path_sync::Coordinate>> Utils::extract_paths(path_sync::mapf_type::NodePtr &current)
{
    std::vector<std::vector<path_sync::Coordinate>> paths;

    std::size_t num_of_agents = current->_state.positions.size();

    while (current != nullptr)
    {
        for (int idx = 0; idx < num_of_agents; ++idx)
        {
            paths[idx].push_back(current->_state.positions[idx]);
        }

        current = current->_parent;
    }

    for (auto &path : paths)
    {
        std::reverse(path.begin(), path.end());
    }

    return paths;
}

} // namespace astar_joint_state
} // namespace mapf
