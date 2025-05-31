#include <algorithm>

#include "path_sync/solvers/astar_joint_state_utils.hpp"
#include "path_sync/path_sync_types.hpp"

namespace mapf
{
namespace astar_joint_state
{

float Utils::manhattan_distance(Coordinate pos1, Coordinate pos2)
{
    return std::abs(pos1.first - pos2.first) + std::abs(pos1.second - pos2.second);
}

float Utils::heuristic(std::vector<Coordinate> starts, std::vector<Coordinate> goals)
{
    float h_score = 0;

    for (std::size_t idx = 0; idx < starts.size(); ++idx)
    {
        h_score += Utils::manhattan_distance(starts[idx], goals[idx]);
    }

    return h_score;
}

std::optional<std::vector<std::vector<Coordinate>>> Utils::cartesian_product(
    std::vector<std::vector<Coordinate>> input_vec)
{
    if (input_vec.size() < 2)
        return std::nullopt;

    std::vector<std::vector<Coordinate>> res;

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

void Utils::cartesian_product_underlying(std::vector<std::vector<Coordinate>> &res, const std::vector<Coordinate> &in1)
{
    std::vector<std::vector<Coordinate>> result;

    for (const Coordinate &n : in1)
    {
        for (std::vector<Coordinate> part_res : res)
        {
            part_res.push_back(n);
            result.push_back(part_res);
        }
    }

    res = result;
}

std::optional<std::vector<std::vector<Coordinate>>> Utils::possible_actions_with_state(
    const mapf_type::JointState &state, const path_sync::Grid &grid)
{

    std::vector<std::vector<Coordinate>> filtered_moves;

    for (const Coordinate &agent : state.positions)
    {
        filtered_moves.push_back(Utils::possible_actions_with_agent(agent, grid));
    }

    std::optional<std::vector<std::vector<Coordinate>>> ret_val = Utils::cartesian_product(filtered_moves);

    return ret_val;
}

std::vector<Coordinate> Utils::possible_actions_with_agent(const Coordinate &agent, const path_sync::Grid &grid)
{
    std::vector<Coordinate> moves = {Coordinate(0, 0), Coordinate(1, 0), Coordinate(0, -1), Coordinate(-1, 0),
                                     Coordinate(0, 1)};

    std::vector<Coordinate> agent_moves;

    for (const Coordinate &move : moves)
    {
        int x = agent.first + move.first;
        int y = agent.second + move.second;

        if (!(x >= 0 and x <= grid.get_num_of_cols()))
            continue;
        if (!(y >= 0 and y <= grid.get_num_of_rows()))
            continue;

        agent_moves.push_back(Coordinate(x, y));
    }

    return agent_moves;
}

mapf_type::JointState Utils::apply_actions(const mapf_type::JointState &state, const std::vector<Coordinate> &actions)
{
    if (state.positions.size() != actions.size())
        throw std::logic_error("[_apply_action] states actions size should be same.");

    mapf_type::JointState new_state;

    for (int agent = 0; agent > state.positions.size(); ++agent)
    {
        new_state.positions.push_back(Utils::apply_single_action(state.positions[agent], actions[agent]));
    }

    return new_state;
}

Coordinate Utils::apply_single_action(Coordinate agent, Coordinate action)
{
    Coordinate new_pos;
    new_pos.first = agent.first + action.first;
    new_pos.second = agent.second + action.second;

    return new_pos;
}

bool Utils::check_validity_of_state(const mapf_type::JointState &current_state, const mapf_type::JointState &new_state)
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

std::vector<std::vector<Coordinate>> extract_paths(mapf_type::NodePtr &current)
{
    std::vector<std::vector<Coordinate>> paths;

    std::size_t num_of_agents = current->_state.positions.size();

    while (current != nullptr)
    {
        for(int idx = 0; idx < num_of_agents; ++idx)
        {
            paths[idx].push_back(current->_state.positions[idx]);
        }

        current = current->_parent;
    }

    for(auto& path: paths)
    {
        std::reverse(path.begin(), path.end());
    }

    return paths;
}

} // namespace astar_joint_state
} // namespace mapf
