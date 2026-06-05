#include "path_sync_core/solvers/jps_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <queue>
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

int octile_distance(ps_coord a, ps_coord b)
{
    int dx = std::abs(a.first - b.first);
    int dy = std::abs(a.second - b.second);
    return std::max(dx, dy);
}

// 8 directions: N, NE, E, SE, S, SW, W, NW
const int dx[] = {0, 1, 1, 1, 0, -1, -1, -1};
const int dy[] = {-1, -1, 0, 1, 1, 1, 0, -1};

ps_coord jump(ps_coord node, int dir_idx, ps_coord goal, const path_sync::MapData &map, int w, int h)
{
    int x = node.first + dx[dir_idx];
    int y = node.second + dy[dir_idx];

    if (x < 0 || x >= w || y < 0 || y >= h || !is_traversable(map, x, y))
        return {-1, -1};

    if (x == goal.first && y == goal.second)
        return {x, y};

    // Check for forced neighbors
    int ddx = dx[dir_idx];
    int ddy = dy[dir_idx];

    if (ddx != 0 && ddy != 0)
    {
        // Diagonal move: check cardinal jumps
        if (jump({x, y}, (dir_idx + 1) % 8, goal, map, w, h).first != -1 ||
            jump({x, y}, (dir_idx + 7) % 8, goal, map, w, h).first != -1)
            return {x, y};

        // Forced neighbor check for diagonal
        if (!is_traversable(map, x - ddx, y) && is_traversable(map, x - ddx, y + ddy))
            return {x, y};
        if (!is_traversable(map, x, y - ddy) && is_traversable(map, x + ddx, y - ddy))
            return {x, y};
    }
    else
    {
        // Cardinal move: forced neighbor check
        if (ddx != 0)
        {
            if (!is_traversable(map, x, y - 1) && is_traversable(map, x + ddx, y - 1))
                return {x, y};
            if (!is_traversable(map, x, y + 1) && is_traversable(map, x + ddx, y + 1))
                return {x, y};
        }
        else
        {
            if (!is_traversable(map, x - 1, y) && is_traversable(map, x - 1, y + ddy))
                return {x, y};
            if (!is_traversable(map, x + 1, y) && is_traversable(map, x + 1, y + ddy))
                return {x, y};
        }
    }

    return jump({x, y}, dir_idx, goal, map, w, h);
}

} // anonymous namespace

namespace path_sync
{
namespace solvers
{
namespace sapf
{

std::string_view JPS_Solver::get_solver_name() const
{
    return solver_name_;
}

std::map<Coordinate, Coordinate> JPS_Solver::solve(const path_sync::MapData &map_data, Coordinate start,
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

    std::map<Coordinate, int> g_score;
    std::map<Coordinate, Coordinate> came_from;

    auto cmp = [&](ps_coord a, ps_coord b)
    {
        int fa = g_score[a] + octile_distance(a, goal);
        int fb = g_score[b] + octile_distance(b, goal);
        if (fa != fb) return fa > fb;
        return a < b;
    };
    std::priority_queue<ps_coord, std::vector<ps_coord>, decltype(cmp)> open(cmp);

    g_score[start] = 0;
    came_from[start] = {-1, -1};
    open.push(start);
    performance_met.peak_open_size = open.size();

    bool found = false;

    while (!open.empty() && !found)
    {
        ps_coord current = open.top();
        open.pop();
        if ((performance_met.cancel_flag && *performance_met.cancel_flag) || performance_met.timed_out()) break;
        performance_met.num_of_nodes_expanded++;

        // Try jumping in all 8 directions from the start, or prune from parent
        for (int d = 0; d < 8; d++)
        {
            ps_coord jp = jump(current, d, goal, map_data, w, h);
            if (jp.first == -1)
                continue;

            int tent_g = g_score[current] + octile_distance(current, jp);

            if (!g_score.count(jp) || tent_g < g_score[jp])
            {
                g_score[jp] = tent_g;
                came_from[jp] = current;
                open.push(jp);
                if (open.size() > performance_met.peak_open_size)
                    performance_met.peak_open_size = open.size();
                performance_met.num_of_nodes_explored++;

                if (jp == goal)
                {
                    found = true;
                    break;
                }
            }
        }
    }

    if (!found)
        came_from.clear();

    performance_met.success = found;
    auto end_time = std::chrono::high_resolution_clock::now();
    performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    return came_from;
}

} // namespace sapf
} // namespace solvers
} // namespace path_sync

#ifdef PATH_SYNC_BUILD_AS_PLUGIN
extern "C"
{

const char *plugin_name() { return "JPS_Solver"; }
bool plugin_is_optimal() { return true; }
bool plugin_is_multi_agent() { return false; }
void *plugin_create() { return new path_sync::solvers::sapf::JPS_Solver(); }
void plugin_destroy(void *p) { delete static_cast<path_sync::solvers::sapf::JPS_Solver *>(p); }

}
#endif
