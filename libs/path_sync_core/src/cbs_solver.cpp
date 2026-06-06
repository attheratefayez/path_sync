#include "path_sync_core/solvers/cbs_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace
{

using ps_coord = path_sync::Coordinate;

int manhattan(ps_coord a, ps_coord b)
{
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
}

bool traversable(const path_sync::MapData &map, int x, int y)
{
    if (x < 0 || x >= map.get_width() || y < 0 || y >= map.get_height())
        return false;
    return map.get_cell_type({x, y}) != path_sync::CellType::WALL;
}

struct LowLevelNode
{
    int x, y;
    int g;
    int h;
    int f() const { return g + h; }
};

struct LowLevelCompare
{
    bool operator()(const LowLevelNode &a, const LowLevelNode &b) const
    {
        if (a.f() != b.f()) return a.f() > b.f();
        if (a.h != b.h) return a.h > b.h;
        return std::tie(a.x, a.y) > std::tie(b.x, b.y);
    }
};

uint64_t state_key(int x, int y, int t)
{
    return (static_cast<uint64_t>(x) << 40)
         | (static_cast<uint64_t>(y) << 20)
         | static_cast<uint64_t>(t);
}

} // anonymous namespace

namespace path_sync::solvers::mapf
{

// ── Low-level A* with vertex/edge constraints ──────────────────────────

std::optional<std::vector<Coordinate>> CBS_Solver::low_level_search(
    const MapData &map, Coordinate start, Coordinate goal,
    const std::vector<CBSConstraint> &constraints, int agent_id) const
{
    // Build constraint lookups for this agent
    std::unordered_map<int, std::vector<std::pair<int, int>>> vert_con;
    std::unordered_map<int, std::vector<std::tuple<int, int, int, int>>> edge_con;

    for (const auto &c : constraints)
    {
        if (c.agent != agent_id && c.agent != -1)
            continue;
        if (c.is_vertex)
            vert_con[c.timestep].emplace_back(c.x, c.y);
        else
            edge_con[c.timestep].emplace_back(c.x, c.y, c.x2, c.y2);
    }

    auto has_vertex = [&](int x, int y, int t) -> bool {
        auto it = vert_con.find(t);
        if (it == vert_con.end()) return false;
        for (auto &p : it->second)
            if (p.first == x && p.second == y) return true;
        return false;
    };

    auto has_edge = [&](int x1, int y1, int x2, int y2, int t) -> bool {
        auto it = edge_con.find(t);
        if (it == edge_con.end()) return false;
        for (auto &[ex1, ey1, ex2, ey2] : it->second)
            if (ex1 == x1 && ey1 == y1 && ex2 == x2 && ey2 == y2) return true;
        return false;
    };

    if (has_vertex(start.first, start.second, 0))
        return std::nullopt;

    static const int dx[] = {0, 1, 0, -1};
    static const int dy[] = {-1, 0, 1, 0};

    std::priority_queue<LowLevelNode, std::vector<LowLevelNode>, LowLevelCompare> open;
    std::unordered_map<uint64_t, int> g_score;
    std::unordered_map<uint64_t, std::pair<int, int>> came_from;
    std::unordered_set<uint64_t> closed;

    auto make_node = [&](int x, int y, int g) -> LowLevelNode {
        return {x, y, g, manhattan({x, y}, goal)};
    };

    uint64_t start_key = state_key(start.first, start.second, 0);
    g_score[start_key] = 0;
    open.push(make_node(start.first, start.second, 0));

    while (!open.empty())
    {
        LowLevelNode cur = open.top();
        open.pop();

        uint64_t cur_key = state_key(cur.x, cur.y, cur.g);
        if (closed.count(cur_key)) continue;
        closed.insert(cur_key);

        // Check vertex constraint at current (skip for goal — can wait there)
        if (cur.g > 0 && has_vertex(cur.x, cur.y, cur.g)
            && !(cur.x == goal.first && cur.y == goal.second))
            continue;

        if (cur.x == goal.first && cur.y == goal.second)
        {
            std::vector<Coordinate> path;
            int cx = cur.x, cy = cur.y, ct = cur.g;
            while (ct >= 0)
            {
                path.emplace_back(cx, cy);
                uint64_t ck = state_key(cx, cy, ct);
                auto it = came_from.find(ck);
                if (it == came_from.end()) break;
                cx = it->second.first;
                cy = it->second.second;
                ct--;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (cur.g >= MAX_TIMESTEP)
            continue;

        // Wait
        if (!has_vertex(cur.x, cur.y, cur.g + 1)
            || (cur.x == goal.first && cur.y == goal.second))
        {
            uint64_t wk = state_key(cur.x, cur.y, cur.g + 1);
            auto it = g_score.find(wk);
            if (it == g_score.end() || cur.g + 1 < it->second)
            {
                g_score[wk] = cur.g + 1;
                came_from[wk] = {cur.x, cur.y};
                open.push(make_node(cur.x, cur.y, cur.g + 1));
            }
        }

        // Move
        for (int d = 0; d < 4; d++)
        {
            int nx = cur.x + dx[d];
            int ny = cur.y + dy[d];
            if (!traversable(map, nx, ny)) continue;

            int nt = cur.g + 1;
            if (has_vertex(nx, ny, nt)) continue;
            if (has_edge(cur.x, cur.y, nx, ny, nt)) continue;

            uint64_t nk = state_key(nx, ny, nt);
            auto it = g_score.find(nk);
            if (it == g_score.end() || nt < it->second)
            {
                g_score[nk] = nt;
                came_from[nk] = {cur.x, cur.y};
                open.push(make_node(nx, ny, nt));
            }
        }
    }

    return std::nullopt;
}

// ── Low-level EPEA* with vertex/edge constraints ────────────────────────

std::optional<std::vector<Coordinate>> CBS_Solver::low_level_search_epea(
    const MapData &map, Coordinate start, Coordinate goal,
    const std::vector<CBSConstraint> &constraints, int agent_id) const
{
    std::unordered_map<int, std::vector<std::pair<int, int>>> vert_con;
    std::unordered_map<int, std::vector<std::tuple<int, int, int, int>>> edge_con;

    for (const auto &c : constraints)
    {
        if (c.agent != agent_id && c.agent != -1)
            continue;
        if (c.is_vertex)
            vert_con[c.timestep].emplace_back(c.x, c.y);
        else
            edge_con[c.timestep].emplace_back(c.x, c.y, c.x2, c.y2);
    }

    auto has_vertex = [&](int x, int y, int t) -> bool {
        auto it = vert_con.find(t);
        if (it == vert_con.end()) return false;
        for (auto &p : it->second)
            if (p.first == x && p.second == y) return true;
        return false;
    };

    auto has_edge = [&](int x1, int y1, int x2, int y2, int t) -> bool {
        auto it = edge_con.find(t);
        if (it == edge_con.end()) return false;
        for (auto &[ex1, ey1, ex2, ey2] : it->second)
            if (ex1 == x1 && ey1 == y1 && ex2 == x2 && ey2 == y2) return true;
        return false;
    };

    if (has_vertex(start.first, start.second, 0))
        return std::nullopt;

    static const int dx[] = {0, 1, 0, -1};
    static const int dy[] = {-1, 0, 1, 0};
    const int NUM_OP = 5;

    struct EPEAEntry
    {
        int x, y, t, f, op_idx;
    };

    auto cmp = [](const EPEAEntry &a, const EPEAEntry &b) {
        if (a.f != b.f) return a.f > b.f;
        if (a.op_idx != b.op_idx) return a.op_idx > b.op_idx;
        if (a.t != b.t) return a.t > b.t;
        if (a.x != b.x) return a.x > b.x;
        return a.y > b.y;
    };

    std::priority_queue<EPEAEntry, std::vector<EPEAEntry>, decltype(cmp)> open(cmp);
    std::unordered_map<uint64_t, int> g_score;
    std::unordered_map<uint64_t, std::pair<int, int>> came_from;
    std::unordered_map<uint64_t, int> op_index;

    uint64_t start_key = state_key(start.first, start.second, 0);
    g_score[start_key] = 0;
    op_index[start_key] = 0;
    open.push({start.first, start.second, 0, manhattan(start, goal), 0});

    while (!open.empty())
    {
        EPEAEntry cur = open.top();
        open.pop();

        uint64_t cur_key = state_key(cur.x, cur.y, cur.t);
        auto git = g_score.find(cur_key);
        if (git == g_score.end() || cur.t > git->second)
            continue;

        if (cur.t > 0 && has_vertex(cur.x, cur.y, cur.t)
            && !(cur.x == goal.first && cur.y == goal.second))
            continue;

        if (cur.x == goal.first && cur.y == goal.second)
        {
            std::vector<Coordinate> path;
            int cx = cur.x, cy = cur.y, ct = cur.t;
            while (ct >= 0)
            {
                path.emplace_back(cx, cy);
                uint64_t ck = state_key(cx, cy, ct);
                auto it = came_from.find(ck);
                if (it == came_from.end()) break;
                cx = it->second.first;
                cy = it->second.second;
                ct--;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (cur.t >= MAX_TIMESTEP)
            continue;

        int &cur_op_idx = op_index[cur_key];
        if (cur.op_idx > 0)
            cur_op_idx = cur.op_idx;

        if (cur_op_idx < NUM_OP)
        {
            bool is_wait = (cur_op_idx == 4);
            int nx = cur.x, ny = cur.y;

            if (!is_wait)
            {
                nx = cur.x + dx[cur_op_idx];
                ny = cur.y + dy[cur_op_idx];
            }

            int nt = cur.t + 1;
            bool valid = false;

            if (is_wait)
            {
                valid = (nx >= 0 && nx < map.get_width()
                      && ny >= 0 && ny < map.get_height());
                if (valid)
                    valid = !has_vertex(nx, ny, nt)
                         || (nx == goal.first && ny == goal.second);
            }
            else
            {
                valid = (nx >= 0 && nx < map.get_width()
                      && ny >= 0 && ny < map.get_height()
                      && map.get_cell_type({nx, ny}) != CellType::WALL);
                if (valid)
                    valid = !has_vertex(nx, ny, nt);
                if (valid)
                    valid = !has_edge(cur.x, cur.y, nx, ny, nt);
            }

            if (valid)
            {
                uint64_t nk = state_key(nx, ny, nt);
                auto it = g_score.find(nk);
                if (it == g_score.end() || nt < it->second)
                {
                    g_score[nk] = nt;
                    came_from[nk] = {cur.x, cur.y};
                    if (op_index.find(nk) == op_index.end())
                        op_index[nk] = 0;
                    open.push({nx, ny, nt, nt + manhattan({nx, ny}, goal), 0});
                }
            }

            cur_op_idx++;

            if (cur_op_idx < NUM_OP)
            {
                int f_val = cur.t + manhattan({cur.x, cur.y}, goal);
                open.push({cur.x, cur.y, cur.t, f_val, cur_op_idx});
            }
        }
    }

    return std::nullopt;
}

// ── Conflict detection ──────────────────────────────────────────────────

std::optional<CBSConflict> CBS_Solver::find_first_conflict(
    const std::vector<std::vector<Coordinate>> &paths) const
{
    if (paths.empty()) return std::nullopt;

    size_t max_len = 0;
    for (const auto &p : paths)
        max_len = std::max(max_len, p.size());

    for (size_t t = 0; t < max_len; t++)
    {
        for (size_t a = 0; a < paths.size(); a++)
        {
            for (size_t b = a + 1; b < paths.size(); b++)
            {
                Coordinate pa = (t < paths[a].size()) ? paths[a][t] : paths[a].back();
                Coordinate pb = (t < paths[b].size()) ? paths[b][t] : paths[b].back();

                if (pa == pb)
                {
                    CBSConflict c;
                    c.agent_a = static_cast<int>(a);
                    c.agent_b = static_cast<int>(b);
                    c.x = pa.first; c.y = pa.second;
                    c.timestep = static_cast<int>(t);
                    c.is_vertex = true;
                    return c;
                }

                if (t > 0)
                {
                    Coordinate pa_prev = paths[a][t - 1];
                    Coordinate pb_prev = paths[b][t - 1];
                    if (pa == pb_prev && pb == pa_prev)
                    {
                        CBSConflict c;
                        c.agent_a = static_cast<int>(a);
                        c.agent_b = static_cast<int>(b);
                        c.x = pa_prev.first; c.y = pa_prev.second;
                        c.x2 = pa.first; c.y2 = pa.second;
                        c.timestep = static_cast<int>(t);
                        c.is_vertex = false;
                        return c;
                    }
                }
            }
        }
    }
    return std::nullopt;
}

// ── Validation and helpers ──────────────────────────────────────────────

bool CBS_Solver::validate_paths_against_constraints(
    const std::vector<std::vector<Coordinate>> &paths,
    const std::vector<CBSConstraint> &constraints) const
{
    for (const auto &c : constraints)
    {
        if (c.agent < 0 || c.agent >= static_cast<int>(paths.size()))
            continue;
        const auto &path = paths[c.agent];
        if (c.timestep >= static_cast<int>(path.size()))
            continue;
        if (path[c.timestep] == Coordinate{c.x, c.y})
            return false;
    }
    return true;
}

int CBS_Solver::compute_soc(const std::vector<std::vector<Coordinate>> &paths) const
{
    int soc = 0;
    for (const auto &p : paths)
        soc += static_cast<int>(p.size()) - 1;
    return soc;
}

// ── Main CBS solve loop ─────────────────────────────────────────────────

std::optional<std::vector<std::vector<Coordinate>>> CBS_Solver::solve(
    const MapData &map_data,
    std::vector<Coordinate> starts,
    std::vector<Coordinate> goals,
    PerformanceMetrics &performance_met)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    int n_agents = static_cast<int>(starts.size());

    int ct_nodes_created = 0;
    int ct_nodes_expanded = 0;
    int conflicts_found = 0;

    // Track last unresolved conflict for failure reporting
    std::optional<CBSConflict> last_conflict;

    // Step 1: initial paths (no constraints)
    std::vector<std::vector<Coordinate>> initial_paths(n_agents);
    for (int i = 0; i < n_agents; i++)
    {
        auto path = use_epea_
            ? low_level_search_epea(map_data, starts[i], goals[i], {}, i)
            : low_level_search(map_data, starts[i], goals[i], {}, i);
        if (!path)
        {
            performance_met.success = false;
            if (performance_met.mapf_metrics)
                performance_met.mapf_metrics->failure_reason = MAFailureReason::PATH_NOT_FOUND;
            performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - start_time);
            return std::nullopt;
        }
        initial_paths[i] = std::move(*path);
    }

    // Step 2: priority queue (min-heap by f = cost + h)
    auto cmp = [](const std::shared_ptr<CBSNode> &a, const std::shared_ptr<CBSNode> &b) {
        int fa = a->cost + a->h;
        int fb = b->cost + b->h;
        if (fa != fb) return fa > fb;
        if (a->h != b->h) return a->h > b->h;
        return a->depth > b->depth;
    };
    std::priority_queue<std::shared_ptr<CBSNode>,
                        std::vector<std::shared_ptr<CBSNode>>,
                        decltype(cmp)> open(cmp);

    auto root = std::make_shared<CBSNode>();
    root->paths = initial_paths;
    root->cost = compute_soc(initial_paths);
    root->h = use_cbsh_ ? compute_cg_heuristic(initial_paths) : 0;
    root->depth = 0;
    open.push(root);
    ct_nodes_created++;

    while (!open.empty())
    {
        if ((performance_met.cancel_flag && *performance_met.cancel_flag) || performance_met.timed_out())
        {
            performance_met.success = false;
            if (performance_met.mapf_metrics)
            {
                performance_met.mapf_metrics->failure_reason =
                    (performance_met.cancel_flag && *performance_met.cancel_flag)
                    ? MAFailureReason::CANCELLED
                    : MAFailureReason::TIMEOUT;
                if (last_conflict.has_value())
                {
                    performance_met.mapf_metrics->last_conflict_agent_a = last_conflict->agent_a;
                    performance_met.mapf_metrics->last_conflict_agent_b = last_conflict->agent_b;
                    performance_met.mapf_metrics->last_conflict_x = last_conflict->x;
                    performance_met.mapf_metrics->last_conflict_y = last_conflict->y;
                    performance_met.mapf_metrics->last_conflict_timestep = last_conflict->timestep;
                }
            }
            performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - start_time);
            return std::nullopt;
        }

        auto N = open.top();
        open.pop();
        ct_nodes_expanded++;

        if (!validate_paths_against_constraints(N->paths, N->constraints))
            continue;

        auto conflict = find_first_conflict(N->paths);
        if (!conflict.has_value())
        {
            // Solution found
            performance_met.success = true;
            performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - start_time);
            performance_met.sum_of_costs = N->cost;
            performance_met.makespan = 0;
            for (const auto &p : N->paths)
                performance_met.makespan = std::max(performance_met.makespan,
                    static_cast<decltype(performance_met.makespan)>(p.size() - 1));
            performance_met.path_length = static_cast<int>(N->paths[0].size());

            if (performance_met.mapf_metrics)
            {
                performance_met.mapf_metrics->joint_states_expanded = ct_nodes_expanded;
                performance_met.mapf_metrics->solution_depth = N->depth;
                performance_met.mapf_metrics->conflicts_detected = conflicts_found;
                size_t max_ts = 0;
                for (const auto &p : N->paths)
                    max_ts = std::max(max_ts, p.size());
                performance_met.mapf_metrics->max_timestep_reached = static_cast<int>(max_ts);
            }
            return N->paths;
        }

        conflicts_found++;
        last_conflict = conflict;

        // ICBS: classify conflict
        CBSConflictClass cc = CBSConflictClass::NON_CARDINAL;
        if (use_icbs_)
            cc = classify_conflict(*conflict, N->paths, map_data, starts, goals);

        // ICBS: bypass attempt for non-cardinal
        if (use_icbs_ && cc == CBSConflictClass::NON_CARDINAL)
        {
            auto bypass = try_bypass(map_data, starts, goals, *N, *conflict);
            if (bypass.has_value())
            {
                performance_met.success = true;
                performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::high_resolution_clock::now() - start_time);
                performance_met.sum_of_costs = compute_soc(*bypass);
                performance_met.makespan = 0;
                for (const auto &p : *bypass)
                    performance_met.makespan = std::max(performance_met.makespan,
                        static_cast<decltype(performance_met.makespan)>(p.size() - 1));

                if (performance_met.mapf_metrics)
                {
                    performance_met.mapf_metrics->joint_states_expanded = ct_nodes_expanded;
                    performance_met.mapf_metrics->solution_depth = N->depth;
                    performance_met.mapf_metrics->conflicts_detected = conflicts_found;
                    size_t max_ts = 0;
                    for (const auto &p : *bypass)
                        max_ts = std::max(max_ts, p.size());
                    performance_met.mapf_metrics->max_timestep_reached = static_cast<int>(max_ts);
                }
                return bypass;
            }
        }

        // Split
        auto make_child = [&](int agent) -> std::shared_ptr<CBSNode> {
            auto child = std::make_shared<CBSNode>();
            child->constraints = N->constraints;
            child->depth = N->depth + 1;

            CBSConstraint con;
            con.agent = agent;
            con.x = conflict->x;
            con.y = conflict->y;
            con.timestep = conflict->timestep;
            child->constraints.push_back(con);

            child->paths = N->paths;
            auto new_path = use_epea_
                ? low_level_search_epea(map_data, starts[agent], goals[agent],
                                        child->constraints, agent)
                : low_level_search(map_data, starts[agent], goals[agent],
                                   child->constraints, agent);
            if (!new_path) return std::shared_ptr<CBSNode>(nullptr);
            child->paths[agent] = std::move(*new_path);
            child->cost = compute_soc(child->paths);
            child->h = use_cbsh_ ? compute_cg_heuristic(child->paths) : 0;
            return child;
        };

        // ICBS: cardinal → split both; semi → split affected; non → split both
        // Plain CBS (no ICBS): always split both
        if (use_icbs_ && cc == CBSConflictClass::SEMI_CARDINAL)
        {
            CBSConstraint test;
            test.agent = conflict->agent_a;
            test.x = conflict->x; test.y = conflict->y;
            test.timestep = conflict->timestep;
            auto test_path = use_epea_
                ? low_level_search_epea(map_data, starts[conflict->agent_a],
                                        goals[conflict->agent_a], {test},
                                        conflict->agent_a)
                : low_level_search(map_data, starts[conflict->agent_a],
                                   goals[conflict->agent_a], {test},
                                   conflict->agent_a);
            int orig = static_cast<int>(N->paths[conflict->agent_a].size()) - 1;
            bool a_affected = !test_path.has_value()
                              || (static_cast<int>(test_path->size()) - 1 > orig);

            int split_agent = a_affected ? conflict->agent_a : conflict->agent_b;
            auto ch = make_child(split_agent);
            if (ch) { open.push(ch); ct_nodes_created++;
                if (performance_met.mapf_metrics) performance_met.mapf_metrics->conflicts_resolved++; }
        }
        else
        {
            auto ca = make_child(conflict->agent_a);
            if (ca) { open.push(ca); ct_nodes_created++;
                if (performance_met.mapf_metrics) performance_met.mapf_metrics->conflicts_resolved++; }
            auto cb = make_child(conflict->agent_b);
            if (cb) { open.push(cb); ct_nodes_created++;
                if (performance_met.mapf_metrics) performance_met.mapf_metrics->conflicts_resolved++; }
        }
    }

    performance_met.success = false;
    if (performance_met.mapf_metrics)
    {
        performance_met.mapf_metrics->failure_reason = MAFailureReason::CT_EXHAUSTED;
        if (last_conflict.has_value())
        {
            performance_met.mapf_metrics->last_conflict_agent_a = last_conflict->agent_a;
            performance_met.mapf_metrics->last_conflict_agent_b = last_conflict->agent_b;
            performance_met.mapf_metrics->last_conflict_x = last_conflict->x;
            performance_met.mapf_metrics->last_conflict_y = last_conflict->y;
            performance_met.mapf_metrics->last_conflict_timestep = last_conflict->timestep;
        }
    }
    performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now() - start_time);
    return std::nullopt;
}

} // namespace path_sync::solvers::mapf

