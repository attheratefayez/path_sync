#include "path_sync_core/solvers/dstar_lite_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/path_sync_types.hpp"

namespace
{

using ps_coord = path_sync::Coordinate;
using Key = path_sync::solvers::sapf::DStarKey;

bool is_traversable(const path_sync::MapData &map, int x, int y)
{
    return map.get_cell_type({x, y}) != path_sync::CellType::WALL;
}

float octile_heuristic(ps_coord a, ps_coord b)
{
    float dx = static_cast<float>(std::abs(a.first - b.first));
    float dy = static_cast<float>(std::abs(a.second - b.second));
    return dx + dy + (std::sqrt(2.0f) - 2.0f) * std::min(dx, dy);
}

} // anonymous namespace

namespace path_sync
{
namespace solvers
{
namespace sapf
{

std::string_view DStar_Lite_Solver::get_solver_name() const
{
    return solver_name_;
}

std::map<Coordinate, Coordinate> DStar_Lite_Solver::solve(const path_sync::MapData &map_data, Coordinate start,
                                                          Coordinate goal, PerformanceMetrics &performance_met)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    const int w = map_data.get_width();
    const int h = map_data.get_height();

    if (!is_traversable(map_data, start.first, start.second) ||
        !is_traversable(map_data, goal.first, goal.second))
        return {};

    if (start == goal)
    {
        std::map<Coordinate, Coordinate> result;
        result[start] = {-1, -1};
        return result;
    }

    // 8-directional movement
    const int ndx[] = {0, 1, 1, 1, 0, -1, -1, -1};
    const int ndy[] = {-1, -1, 0, 1, 1, 1, 0, -1};
    const float move_cost[] = {1.0f, 1.414f, 1.0f, 1.414f, 1.0f, 1.414f, 1.0f, 1.414f};

    auto get_neighbors = [&](ps_coord c) -> std::vector<std::pair<ps_coord, float>>
    {
        std::vector<std::pair<ps_coord, float>> result;
        for (int d = 0; d < 8; d++)
        {
            int nx = c.first + ndx[d];
            int ny = c.second + ndy[d];
            if (nx >= 0 && nx < w && ny >= 0 && ny < h && is_traversable(map_data, nx, ny))
            {
                result.push_back({{nx, ny}, move_cost[d]});
            }
        }
        return result;
    };

    // g and rhs values
    std::map<ps_coord, float> g, rhs;
    // Priority queue: we use a multiset-style approach
    struct QueueEntry
    {
        ps_coord pos;
        Key key;
    };
    auto cmp = [](const QueueEntry &a, const QueueEntry &b)
    {
        if (a.key.k0 != b.key.k0) return a.key.k0 < b.key.k0;
        if (a.key.k1 != b.key.k1) return a.key.k1 < b.key.k1;
        return a.pos < b.pos;
    };
    std::set<QueueEntry, decltype(cmp)> U(cmp);
    std::set<ps_coord> in_U;

    float km = 0;

    auto calc_key = [&](ps_coord s) -> Key
    {
        float val = std::min(g[s], rhs[s]);
        return {val + octile_heuristic(start, s) + km, val};
    };

    auto update_vertex = [&](ps_coord u)
    {
        if (u != goal)
        {
            float min_rhs = std::numeric_limits<float>::infinity();
            for (auto &[succ, cost] : get_neighbors(u))
            {
                min_rhs = std::min(min_rhs, cost + g[succ]);
            }
            rhs[u] = min_rhs;
        }

        if (in_U.count(u))
        {
            U.erase({u, calc_key(u)});
            in_U.erase(u);
        }

        if (g[u] != rhs[u])
        {
            U.insert({u, calc_key(u)});
            in_U.insert(u);
        }
    };

    // Initialize
    rhs[goal] = 0;
    g[goal] = std::numeric_limits<float>::infinity();
    rhs[start] = std::numeric_limits<float>::infinity();
    g[start] = std::numeric_limits<float>::infinity();

    // Initialize all nodes (lazy - we initialize on first encounter)
    // Insert goal into U
    {
        Key k = calc_key(goal);
        U.insert({goal, k});
        in_U.insert(goal);
    }

    // ComputeShortestPath
    auto top_key = [&]() -> Key
    {
        return U.begin()->key;
    };

    auto compute_shortest_path = [&]()
    {
        while (!U.empty())
        {
            Key k_old = top_key();
            Key k_start = calc_key(start);

            if ((k_old < k_start) == false && rhs[start] == g[start])
                break;

            QueueEntry u_entry = *U.begin();
            ps_coord u = u_entry.pos;
            U.erase(U.begin());
            in_U.erase(u);

            Key k_new = calc_key(u);

            if (k_old < k_new)
            {
                U.insert({u, k_new});
                in_U.insert(u);
            }
            else if (g[u] > rhs[u])
            {
                g[u] = rhs[u];
                for (auto &[pred, cost] : get_neighbors(u))
                {
                    // Ensure g and rhs are initialized
                    if (!g.count(pred))
                        g[pred] = std::numeric_limits<float>::infinity();
                    if (!rhs.count(pred))
                        rhs[pred] = std::numeric_limits<float>::infinity();
                    update_vertex(pred);
                }
            }
            else
            {
                float g_old = g[u];
                g[u] = std::numeric_limits<float>::infinity();

                for (auto &[pred, cost] : get_neighbors(u))
                {
                    if (!g.count(pred))
                        g[pred] = std::numeric_limits<float>::infinity();
                    if (!rhs.count(pred))
                        rhs[pred] = std::numeric_limits<float>::infinity();
                    update_vertex(pred);
                }
                update_vertex(u);
            }
        }
    };

    compute_shortest_path();

    // Reconstruct path using best-first from start to goal
    std::map<Coordinate, Coordinate> came_from;

    if (rhs[start] == std::numeric_limits<float>::infinity())
        return {};

    ps_coord current = start;
    came_from[start] = {-1, -1};
    performance_met.num_of_nodes_explored = 0;

    while (current != goal)
    {
        auto neighbors = get_neighbors(current);
        if (neighbors.empty())
        {
            came_from.clear();
            break;
        }

        ps_coord best;
        float best_val = std::numeric_limits<float>::infinity();
        for (auto &[nb, cost] : neighbors)
        {
            if (!g.count(nb))
                g[nb] = std::numeric_limits<float>::infinity();
            float val = cost + g[nb];
            if (val < best_val)
            {
                best_val = val;
                best = nb;
            }
        }

        if (best_val == std::numeric_limits<float>::infinity())
        {
            came_from.clear();
            break;
        }

        came_from[best] = current;
        current = best;
        performance_met.num_of_nodes_explored++;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    performance_met.runtime = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    return came_from;
}

} // namespace sapf
} // namespace solvers
} // namespace path_sync
