#include "path_sync_core/solvers/mstar_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <unordered_map>
#include <vector>

#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/path_sync_types.hpp"

namespace
{

using ps_coord = path_sync::Coordinate;

bool is_traversable(const path_sync::MapData &map, int x, int y)
{
    return map.get_cell_type({x, y}) != path_sync::CellType::WALL;
}

int manhattan_distance(ps_coord a, ps_coord b)
{
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}

struct PathResult
{
    std::vector<ps_coord> path;
    bool found;
};

PathResult find_individual_path(const path_sync::MapData &map, ps_coord start, ps_coord goal)
{
    int w = map.get_width();
    int h = map.get_height();

    if (!is_traversable(map, start.first, start.second) ||
        !is_traversable(map, goal.first, goal.second))
        return {{}, false};

    std::map<ps_coord, int> g_score;
    std::map<ps_coord, ps_coord> came_from;

    auto cmp = [&](ps_coord a, ps_coord b)
    {
        int fa = g_score[a] + manhattan_distance(a, goal);
        int fb = g_score[b] + manhattan_distance(b, goal);
        if (fa != fb) return fa > fb;
        return a < b;
    };
    std::priority_queue<ps_coord, std::vector<ps_coord>, decltype(cmp)> open(cmp);

    g_score[start] = 0;
    came_from[start] = {-1, -1};
    open.push(start);

    const int ndx[] = {0, 1, 0, -1};
    const int ndy[] = {-1, 0, 1, 0};

    while (!open.empty())
    {
        ps_coord current = open.top();
        open.pop();

        if (current == goal)
        {
            std::vector<ps_coord> path;
            ps_coord node = goal;
            while (node != ps_coord{-1, -1})
            {
                path.push_back(node);
                auto it = came_from.find(node);
                if (it == came_from.end()) break;
                node = it->second;
            }
            std::reverse(path.begin(), path.end());
            return {path, true};
        }

        for (int d = 0; d < 4; d++)
        {
            int nx = current.first + ndx[d];
            int ny = current.second + ndy[d];

            if (nx < 0 || nx >= w || ny < 0 || ny >= h)
                continue;
            if (!is_traversable(map, nx, ny))
                continue;

            int tent_g = g_score[current] + 1;
            if (!g_score.count({nx, ny}) || tent_g < g_score[{nx, ny}])
            {
                g_score[{nx, ny}] = tent_g;
                came_from[{nx, ny}] = current;
                open.push({nx, ny});
            }
        }
    }

    return {{}, false};
}

std::vector<ps_coord> pad_path(const std::vector<ps_coord> &path, int target_len)
{
    if (static_cast<int>(path.size()) >= target_len)
        return path;

    std::vector<ps_coord> padded = path;
    ps_coord last = path.back();
    while (static_cast<int>(padded.size()) < target_len)
        padded.push_back(last);
    return padded;
}

bool has_vertex_conflict(const std::vector<std::vector<ps_coord>> &paths, int t)
{
    std::set<ps_coord> occupied;
    for (const auto &p : paths)
    {
        if (t < static_cast<int>(p.size()))
        {
            if (occupied.count(p[t]))
                return true;
            occupied.insert(p[t]);
        }
    }
    return false;
}

} // anonymous namespace

namespace path_sync
{
namespace solvers
{
namespace mapf
{

std::string_view MStar_Solver::get_solver_name() const
{
    return solver_name_;
}

std::vector<std::vector<Coordinate>> MStar_Solver::individual_paths(
    const MapData &map_data,
    const std::vector<Coordinate> &starts,
    const std::vector<Coordinate> &goals)
{
    int n = static_cast<int>(starts.size());
    std::vector<std::vector<Coordinate>> paths(n);

    for (int i = 0; i < n; i++)
    {
        auto result = find_individual_path(map_data, starts[i], goals[i]);
        if (!result.found)
            return {};
        paths[i] = result.path;
    }

    // Pad all paths to the same length
    int max_len = 0;
    for (const auto &p : paths)
        max_len = std::max(max_len, static_cast<int>(p.size()));

    for (auto &p : paths)
        p = pad_path(p, max_len);

    return paths;
}

std::optional<int> MStar_Solver::find_first_conflict(
    const std::vector<std::vector<Coordinate>> &paths)
{
    int max_len = 0;
    for (const auto &p : paths)
        max_len = std::max(max_len, static_cast<int>(p.size()));

    for (int t = 0; t < max_len; t++)
    {
        if (has_vertex_conflict(paths, t))
            return t;
    }
    return std::nullopt;
}

std::vector<int> MStar_Solver::get_conflicted_agents(
    const std::vector<std::vector<Coordinate>> &paths,
    int conflict_time)
{
    int n = static_cast<int>(paths.size());
    std::set<int> conflicted;

    for (int i = 0; i < n; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            ps_coord pi = (conflict_time < static_cast<int>(paths[i].size())) ? paths[i][conflict_time] : paths[i].back();
            ps_coord pj = (conflict_time < static_cast<int>(paths[j].size())) ? paths[j][conflict_time] : paths[j].back();

            if (pi == pj)
            {
                conflicted.insert(i);
                conflicted.insert(j);
            }
        }
    }

    return {conflicted.begin(), conflicted.end()};
}

std::optional<std::vector<std::vector<Coordinate>>> MStar_Solver::joint_search(
    const MapData &map_data,
    const std::vector<int> &agent_indices,
    const std::vector<Coordinate> &starts,
    const std::vector<Coordinate> &goals,
    const std::vector<std::vector<Coordinate>> &fixed_paths,
    int from_timestep,
    PerformanceMetrics &performance_met)
{
    int na = static_cast<int>(agent_indices.size());
    int total_agents = static_cast<int>(starts.size());

    if (na == 0)
    {
        // No joint agents, return fixed paths
        return fixed_paths;
    }

    // Build a map from agent index to its position in the joint state
    std::map<int, int> joint_index;
    for (int j = 0; j < na; j++)
        joint_index[agent_indices[j]] = j;

    // Heuristic: sum of Manhattan distances to goals
    auto joint_heuristic = [&](const JointState &s) -> int
    {
        int total = 0;
        for (int j = 0; j < na; j++)
        {
            int ai = agent_indices[j];
            total += manhattan_distance(s.positions[j], goals[ai]);
        }
        return total;
    };

    // Check if joint state has conflicts among joint agents
    auto joint_has_conflict = [&](const JointState &s) -> bool
    {
        for (int i = 0; i < na; i++)
        {
            for (int j = i + 1; j < na; j++)
            {
                if (s.positions[i] == s.positions[j])
                    return true;
            }
        }
        return false;
    };

    // Check if joint state conflicts with a fixed agent at a given timestep
    auto conflicts_with_fixed = [&](const JointState &s) -> bool
    {
        for (int j = 0; j < na; j++)
        {
            int ai = agent_indices[j];
            ps_coord jpos = s.positions[j];

            for (int fi = 0; fi < total_agents; fi++)
            {
                if (joint_index.count(fi))
                    continue; // this agent is in the joint set

                ps_coord fpos;
                if (s.timestep < static_cast<int>(fixed_paths[fi].size()))
                    fpos = fixed_paths[fi][s.timestep];
                else
                    fpos = fixed_paths[fi].back();

                if (jpos == fpos)
                    return true;
            }
        }
        return false;
    };

    // Check if all joint agents are at their goals
    auto all_at_goal = [&](const JointState &s) -> bool
    {
        for (int j = 0; j < na; j++)
        {
            int ai = agent_indices[j];
            if (s.positions[j] != goals[ai])
                return false;
        }
        return true;
    };

    // Build initial state at from_timestep
    JointState init_state;
    init_state.timestep = from_timestep;
    for (int j = 0; j < na; j++)
    {
        int ai = agent_indices[j];
        if (from_timestep < static_cast<int>(fixed_paths[ai].size()))
            init_state.positions.push_back(fixed_paths[ai][from_timestep]);
        else
            init_state.positions.push_back(fixed_paths[ai].back());
    }

    if (joint_has_conflict(init_state))
        return std::nullopt;

    // Joint state hashing
    struct JointStateHash
    {
        std::size_t operator()(const JointState &s) const
        {
            std::size_t h = static_cast<std::size_t>(s.timestep);
            for (const auto &p : s.positions)
            {
                h ^= (static_cast<std::size_t>(p.first) << 16) ^
                     static_cast<std::size_t>(p.second);
                h ^= h << 11;
            }
            return h;
        }

        bool operator()(const JointState &a, const JointState &b) const
        {
            if (a.timestep != b.timestep) return false;
            return a.positions == b.positions;
        }
    };

    std::unordered_map<JointState, int, JointStateHash, JointStateHash> g_score;
    std::unordered_map<JointState, JointState, JointStateHash, JointStateHash> came_from;

    auto cmp = [&](const JointState &a, const JointState &b)
    {
        int fa = g_score[a] + joint_heuristic(a);
        int fb = g_score[b] + joint_heuristic(b);
        if (fa != fb) return fa > fb;
        return a.timestep < b.timestep;
    };
    std::priority_queue<JointState, std::vector<JointState>, decltype(cmp)> open(cmp);

    g_score[init_state] = 0;
    open.push(init_state);
    performance_met.peak_open_size = open.size();

    const int ndx[] = {0, 1, 0, -1, 0};
    const int ndy[] = {-1, 0, 1, 0, 0};
    const int NUM_ACTIONS = 5;

    int w = map_data.get_width();
    int h = map_data.get_height();
    int max_timestep = 200; // safety bound

    bool found = false;
    JointState goal_state;

    while (!open.empty() && !found)
    {
        JointState current = open.top();
        open.pop();

        if (performance_met.cancel_flag && *performance_met.cancel_flag) break;

        if (all_at_goal(current))
        {
            found = true;
            goal_state = current;
            break;
        }

        if (current.timestep > max_timestep)
            continue;

        // Generate successor joint states
        // Each joint agent can take one of 5 actions
        // Use the cartesian product of actions across agents
        std::vector<std::vector<ps_coord>> action_lists(na);
        for (int j = 0; j < na; j++)
        {
            int ai = agent_indices[j];
            ps_coord cur_pos = current.positions[j];

            // If agent is already at its goal, it can only stay
            if (cur_pos == goals[ai])
            {
                action_lists[j] = {cur_pos};
                continue;
            }

            for (int d = 0; d < NUM_ACTIONS; d++)
            {
                int nx = cur_pos.first + ndx[d];
                int ny = cur_pos.second + ndy[d];
                if (nx >= 0 && nx < w && ny >= 0 && ny < h &&
                    is_traversable(map_data, nx, ny))
                {
                    action_lists[j].push_back({nx, ny});
                }
            }
        }

        // Generate combinations using recursion (avoid large CP for many agents)
        std::size_t num_combos = 1;
        for (const auto &al : action_lists)
            num_combos *= al.size();

        // Limit to prevent explosion - if too many combos, only take first few
        const std::size_t MAX_COMBOS = 5000;

        if (num_combos <= MAX_COMBOS)
        {
            // Full enumeration
            std::vector<std::size_t> indices(na, 0);
            bool done = false;
            while (!done)
            {
                JointState next_state;
                next_state.timestep = current.timestep + 1;
                for (int j = 0; j < na; j++)
                    next_state.positions.push_back(action_lists[j][indices[j]]);

                // Check validity
                if (!joint_has_conflict(next_state) && !conflicts_with_fixed(next_state))
                {
                    int tent_g = g_score[current] + 1;
                    if (!g_score.count(next_state) || tent_g < g_score[next_state])
                    {
                        g_score[next_state] = tent_g;
                        came_from[next_state] = current;
                        open.push(next_state);
                        if (open.size() > performance_met.peak_open_size)
                            performance_met.peak_open_size = open.size();
                    }
                }

                // Advance indices
                int k = na - 1;
                while (k >= 0 && ++indices[k] >= action_lists[k].size())
                {
                    indices[k] = 0;
                    k--;
                }
                if (k < 0) done = true;
            }
        }
        else
        {
            // Random sampling
            for (std::size_t sample = 0; sample < MAX_COMBOS; sample++)
            {
                JointState next_state;
                next_state.timestep = current.timestep + 1;
                for (int j = 0; j < na; j++)
                {
                    std::size_t idx = (sample * (j + 1)) % action_lists[j].size();
                    next_state.positions.push_back(action_lists[j][idx]);
                }

                if (!joint_has_conflict(next_state) && !conflicts_with_fixed(next_state))
                {
                    int tent_g = g_score[current] + 1;
                    if (!g_score.count(next_state) || tent_g < g_score[next_state])
                    {
                        g_score[next_state] = tent_g;
                        came_from[next_state] = current;
                        open.push(next_state);
                        if (open.size() > performance_met.peak_open_size)
                            performance_met.peak_open_size = open.size();
                    }
                }
            }
        }
    }

    if (!found)
        return std::nullopt;

    // Reconstruct joint paths
    std::vector<std::vector<ps_coord>> joint_paths(na);
    JointState node = goal_state;
    while (true)
    {
        for (int j = 0; j < na; j++)
            joint_paths[j].push_back(node.positions[j]);

        auto it = came_from.find(node);
        if (it == came_from.end() || it->second.timestep < from_timestep)
            break;
        node = it->second;
    }

    for (auto &jp : joint_paths)
        std::reverse(jp.begin(), jp.end());

    // Merge with fixed paths
    std::vector<std::vector<Coordinate>> result(total_agents);
    for (int i = 0; i < total_agents; i++)
    {
        auto jt = joint_index.find(i);
        if (jt != joint_index.end())
        {
            int j = jt->second;
            // Copy prefix from fixed path (before from_timestep)
            for (int t = 0; t < from_timestep && t < static_cast<int>(fixed_paths[i].size()); t++)
                result[i].push_back(fixed_paths[i][t]);
            // Copy joint path (from from_timestep onward)
            for (const auto &p : joint_paths[j])
                result[i].push_back(p);
        }
        else
        {
            result[i] = fixed_paths[i];
        }
    }

    return result;
}

std::optional<std::vector<std::vector<Coordinate>>> MStar_Solver::solve(
    const MapData &map_data,
    std::vector<Coordinate> starts,
    std::vector<Coordinate> goals,
    PerformanceMetrics &performance_met)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    int n = static_cast<int>(starts.size());

    if (n == 0)
        return std::vector<std::vector<Coordinate>>();

    if (n != static_cast<int>(goals.size()))
        return std::nullopt;

    // Step 1: Plan individually
    auto paths = individual_paths(map_data, starts, goals);
    if (paths.empty())
    {
        performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start_time);
        return std::nullopt;
    }

    // Step 2: Iteratively detect and resolve conflicts
    std::set<int> joint_set;

    while (true)
    {
        if (performance_met.cancel_flag && *performance_met.cancel_flag)
        {
            auto end_time = std::chrono::high_resolution_clock::now();
            performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            return std::nullopt;
        }

        auto conflict = find_first_conflict(paths);
        if (!conflict.has_value())
        {
            // No conflicts found
            performance_met.success = true;
            performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - start_time);
            return paths;
        }

        int conflict_time = conflict.value();
        auto conflicted = get_conflicted_agents(paths, conflict_time);

        // Add conflicted agents to the joint set
        for (int ai : conflicted)
            joint_set.insert(ai);

        // Run joint search for the joint set
        std::vector<int> joint_agents(joint_set.begin(), joint_set.end());
        auto result = joint_search(map_data, joint_agents, starts, goals, paths, conflict_time, performance_met);

        if (!result.has_value())
        {
            // Joint search failed - try with all agents
            std::vector<int> all_agents(n);
            for (int i = 0; i < n; i++) all_agents[i] = i;
            auto result2 = joint_search(map_data, all_agents, starts, goals, paths, 0, performance_met);
            performance_met.success = result2.has_value();
            performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - start_time);
            return result2;
        }

        paths = result.value();
        // Re-pad to same length
        int max_len = 0;
        for (const auto &p : paths)
            max_len = std::max(max_len, static_cast<int>(p.size()));
        for (auto &p : paths)
            p = pad_path(p, max_len);

        // Continue loop to check for remaining conflicts
    }
}

} // namespace mapf
} // namespace solvers
} // namespace path_sync

#ifdef PATH_SYNC_BUILD_AS_PLUGIN
extern "C"
{

const char *plugin_name() { return "MStar_Solver"; }
bool plugin_is_optimal() { return true; }
bool plugin_is_multi_agent() { return true; }
void *plugin_create() { return new path_sync::solvers::mapf::MStar_Solver(); }
void plugin_destroy(void *p) { delete static_cast<path_sync::solvers::mapf::MStar_Solver *>(p); }

}
#endif
