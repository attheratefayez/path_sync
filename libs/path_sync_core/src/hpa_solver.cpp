#include "path_sync_core/solvers/hpa_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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

std::vector<ps_coord> local_a_star(const path_sync::MapData &map, ps_coord start, ps_coord goal,
                                   const std::set<ps_coord> &blocked)
{
    int w = map.get_width();
    int h = map.get_height();

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

    const int dx[] = {0, 1, 0, -1};
    const int dy[] = {-1, 0, 1, 0};

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
            return path;
        }

        for (int d = 0; d < 4; d++)
        {
            int nx = current.first + dx[d];
            int ny = current.second + dy[d];

            if (nx < 0 || nx >= w || ny < 0 || ny >= h)
                continue;
            if (!is_traversable(map, nx, ny))
                continue;
            if (blocked.count({nx, ny}))
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

    return {};
}

} // anonymous namespace

namespace path_sync
{
namespace solvers
{
namespace sapf
{

std::string_view HPA_Solver::get_solver_name() const
{
    return solver_name_;
}

int HPA_Solver::get_cluster_id(int x, int y, int w, int h) const
{
    int cx = x / cluster_size_;
    int cy = y / cluster_size_;
    int nc = (w + cluster_size_ - 1) / cluster_size_;
    return cy * nc + cx;
}

std::vector<std::vector<int>> HPA_Solver::build_abstract_graph(
    const MapData &map_data, std::vector<AbstractNode> &nodes) const
{
    int w = map_data.get_width();
    int h = map_data.get_height();
    int nc_x = (w + cluster_size_ - 1) / cluster_size_;
    int nc_y = (h + cluster_size_ - 1) / cluster_size_;

    // Map from cluster border coords to node IDs
    std::map<Coordinate, int> coord_to_node;

    auto add_or_get = [&](Coordinate c) -> int
    {
        auto it = coord_to_node.find(c);
        if (it != coord_to_node.end())
            return it->second;
        int id = static_cast<int>(nodes.size());
        nodes.push_back({c, get_cluster_id(c.first, c.second, w, h)});
        coord_to_node[c] = id;
        return id;
    };

    // Collect entrance points on vertical and horizontal borders
    std::set<Coordinate> entrances;

    // Vertical borders between clusters
    for (int cx = 1; cx < nc_x; cx++)
    {
        int bx = cx * cluster_size_;
        for (int y = 0; y < h; y++)
        {
            if (bx >= w) break;
            if (is_traversable(map_data, bx, y) && is_traversable(map_data, bx - 1, y))
            {
                entrances.insert({bx, y});
                entrances.insert({bx - 1, y});
            }
        }
    }

    // Horizontal borders between clusters
    for (int cy = 1; cy < nc_y; cy++)
    {
        int by = cy * cluster_size_;
        for (int x = 0; x < w; x++)
        {
            if (by >= h) break;
            if (is_traversable(map_data, x, by) && is_traversable(map_data, x, by - 1))
            {
                entrances.insert({x, by});
                entrances.insert({x, by - 1});
            }
        }
    }

    // Build abstract adjacency (adjacency matrix as vector of vectors of edges)
    int n_nodes = static_cast<int>(nodes.size()) + static_cast<int>(entrances.size());
    // We'll add entrances first, then connect them
    std::vector<int> entrance_ids;
    for (const auto &e : entrances)
    {
        int id = add_or_get(e);
        entrance_ids.push_back(id);
    }

    n_nodes = static_cast<int>(nodes.size());
    std::vector<std::vector<int>> adj(n_nodes);

    // Connect adjacent entrance pairs across borders
    for (const auto &e : entrances)
    {
        int id = coord_to_node[e];
        int cx = e.first / cluster_size_;
        int cy = e.second / cluster_size_;

        // Check neighbor cells that might be in adjacent clusters
        const int ndx[] = {0, 1, 0, -1};
        const int ndy[] = {-1, 0, 1, 0};
        for (int d = 0; d < 4; d++)
        {
            int nx = e.first + ndx[d];
            int ny = e.second + ndy[d];
            if (nx < 0 || nx >= w || ny < 0 || ny >= h)
                continue;
            int ncx = nx / cluster_size_;
            int ncy = ny / cluster_size_;
            if (ncx != cx || ncy != cy)
            {
                // Adjacent cluster neighbor
                auto it = coord_to_node.find({nx, ny});
                if (it != coord_to_node.end())
                {
                    adj[id].push_back(it->second);
                    adj[it->second].push_back(id);
                }
            }
        }
    }

    // Intra-cluster connections: for each cluster, connect all entrance pairs
    std::map<int, std::vector<int>> cluster_nodes;
    for (const auto &e : entrances)
    {
        int id = coord_to_node[e];
        int cid = get_cluster_id(e.first, e.second, w, h);
        cluster_nodes[cid].push_back(id);
    }

    for (auto &[cid, ids] : cluster_nodes)
    {
        for (size_t i = 0; i < ids.size(); i++)
        {
            for (size_t j = i + 1; j < ids.size(); j++)
            {
                auto p1 = nodes[ids[i]].pos;
                auto p2 = nodes[ids[j]].pos;
                std::set<ps_coord> blocked;
                auto path = local_a_star(map_data, p1, p2, blocked);
                if (!path.empty())
                {
                    adj[ids[i]].push_back(ids[j]);
                    adj[ids[j]].push_back(ids[i]);
                }
            }
        }
    }

    return adj;
}

std::vector<Coordinate> HPA_Solver::refine_path(
    const MapData &map_data,
    const std::vector<int> &abstract_path,
    const std::vector<AbstractNode> &nodes,
    Coordinate start, Coordinate goal) const
{
    std::vector<Coordinate> full_path;

    std::set<ps_coord> blocked;
    for (size_t i = 0; i + 1 < abstract_path.size(); i++)
    {
        int from_id = abstract_path[i];
        int to_id = abstract_path[i + 1];

        Coordinate a = nodes[from_id].pos;
        Coordinate b = nodes[to_id].pos;

        auto seg = local_a_star(map_data, a, b, blocked);
        if (seg.empty())
            return {};

        if (full_path.empty())
        {
            full_path = seg;
        }
        else
        {
            full_path.insert(full_path.end(), seg.begin() + 1, seg.end());
        }
    }

    return full_path;
}

std::map<Coordinate, Coordinate> HPA_Solver::solve(const path_sync::MapData &map_data, Coordinate start,
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

    // If map is too small for clustering, use local A* directly
    if (w <= cluster_size_ || h <= cluster_size_)
    {
        std::set<ps_coord> blocked;
        auto path = local_a_star(map_data, start, goal, blocked);
        if (path.empty())
            return {};

        std::map<Coordinate, Coordinate> came_from;
        came_from[path[0]] = {-1, -1};
        for (size_t i = 1; i < path.size(); i++)
            came_from[path[i]] = path[i - 1];

        auto end_time = std::chrono::high_resolution_clock::now();
        performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        performance_met.success = true;
        return came_from;
    }

    // Build abstract graph
    std::vector<AbstractNode> abstract_nodes;
    auto abstract_adj = build_abstract_graph(map_data, abstract_nodes);

    // Add start and goal as abstract nodes
    Coordinate start_coord = start;
    Coordinate goal_coord = goal;
    int start_id = static_cast<int>(abstract_nodes.size());
    abstract_nodes.push_back({start_coord, get_cluster_id(start.first, start.second, w, h)});
    int goal_id = static_cast<int>(abstract_nodes.size());
    abstract_nodes.push_back({goal_coord, get_cluster_id(goal.first, goal.second, w, h)});

    // Extend adjacency
    abstract_adj.resize(abstract_nodes.size());

    // Connect start/goal to nearby entrances in their cluster
    int sc = get_cluster_id(start.first, start.second, w, h);
    int gc = get_cluster_id(goal.first, goal.second, w, h);

    for (int i = 0; i < static_cast<int>(abstract_nodes.size()) - 2; i++)
    {
        if (abstract_nodes[i].cluster_id == sc)
        {
            std::set<ps_coord> blocked;
            auto path_to_start = local_a_star(map_data, abstract_nodes[i].pos, start_coord, blocked);
            if (!path_to_start.empty())
            {
                abstract_adj[start_id].push_back(i);
                abstract_adj[i].push_back(start_id);
            }
        }
        if (abstract_nodes[i].cluster_id == gc)
        {
            std::set<ps_coord> blocked;
            auto path_to_goal = local_a_star(map_data, abstract_nodes[i].pos, goal_coord, blocked);
            if (!path_to_goal.empty())
            {
                abstract_adj[goal_id].push_back(i);
                abstract_adj[i].push_back(goal_id);
            }
        }
    }

    // Also directly connect start to goal if same cluster
    if (sc == gc)
    {
        std::set<ps_coord> blocked;
        auto direct_path = local_a_star(map_data, start_coord, goal_coord, blocked);
        if (!direct_path.empty())
        {
            abstract_adj[start_id].push_back(goal_id);
            abstract_adj[goal_id].push_back(start_id);
        }
    }

    // A* on abstract graph
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> abstract_open;
    std::vector<int> abstract_g(abstract_nodes.size(), std::numeric_limits<int>::max());
    std::vector<int> abstract_parent(abstract_nodes.size(), -1);

    abstract_g[start_id] = 0;
    abstract_open.push({0, start_id});
    performance_met.peak_open_size = abstract_open.size();

    bool found = false;

    while (!abstract_open.empty() && !found)
    {
        auto [fg, cur] = abstract_open.top();
        abstract_open.pop();

        if (cur == goal_id)
        {
            found = true;
            break;
        }

        if (fg > abstract_g[cur])
            continue;

        for (int nb : abstract_adj[cur])
        {
            int cost = 1; // uniform cost for abstract edges
            int tent = abstract_g[cur] + cost;
            if (tent < abstract_g[nb])
            {
                abstract_g[nb] = tent;
                abstract_parent[nb] = cur;
                abstract_open.push({tent, nb});
                if (abstract_open.size() > performance_met.peak_open_size)
                    performance_met.peak_open_size = abstract_open.size();
            }
        }
    }

    if (!found)
        return {};

    // Reconstruct abstract path
    std::vector<int> abstract_path;
    for (int v = goal_id; v != -1; v = abstract_parent[v])
        abstract_path.push_back(v);
    std::reverse(abstract_path.begin(), abstract_path.end());

    // Refine
    auto full_path = refine_path(map_data, abstract_path, abstract_nodes, start, goal);

    if (full_path.empty())
        return {};

    // Convert path to came_from map
    std::map<Coordinate, Coordinate> came_from;
    came_from[full_path[0]] = {-1, -1};
    for (size_t i = 1; i < full_path.size(); i++)
        came_from[full_path[i]] = full_path[i - 1];

    auto end_time = std::chrono::high_resolution_clock::now();
    performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    performance_met.success = true;

    return came_from;
}

} // namespace sapf
} // namespace solvers
} // namespace path_sync
