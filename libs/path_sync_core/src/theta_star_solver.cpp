#include "path_sync_core/solvers/theta_star_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
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

float euclidean_distance(ps_coord a, ps_coord b)
{
    int dx = a.first - b.first;
    int dy = a.second - b.second;
    return std::sqrt(dx * dx + dy * dy);
}

bool line_of_sight(ps_coord a, ps_coord b, const path_sync::MapData &map)
{
    int x0 = a.first, y0 = a.second;
    int x1 = b.first, y1 = b.second;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
        if (!is_traversable(map, x0, y0))
            return false;

        if (x0 == x1 && y0 == y1)
            break;

        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }

    return true;
}

} // anonymous namespace

namespace path_sync
{
namespace solvers
{
namespace sapf
{

std::string_view Theta_Star_Solver::get_solver_name() const
{
    return solver_name_;
}

std::map<Coordinate, Coordinate> Theta_Star_Solver::solve(const path_sync::MapData &map_data, Coordinate start,
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

    const Coordinate UNVISITED{-2, -2};
    std::vector<std::vector<float>> g_score(h, std::vector<float>(w, std::numeric_limits<float>::infinity()));
    std::vector<std::vector<Coordinate>> came_from(h, std::vector<Coordinate>(w, UNVISITED));
    std::vector<std::vector<bool>> closed(h, std::vector<bool>(w, false));

    auto cmp = [&](ps_coord a, ps_coord b)
    {
        float fa = g_score[a.second][a.first] + euclidean_distance(a, goal);
        float fb = g_score[b.second][b.first] + euclidean_distance(b, goal);
        if (fa != fb) return fa > fb;
        return a < b;
    };
    std::priority_queue<ps_coord, std::vector<ps_coord>, decltype(cmp)> open(cmp);

    g_score[start.second][start.first] = 0;
    came_from[start.second][start.first] = start;
    open.push(start);
    performance_met.peak_open_size = open.size();

    const int dx[] = {0, 1, 1, 1, 0, -1, -1, -1};
    const int dy[] = {-1, -1, 0, 1, 1, 1, 0, -1};

    bool found = false;

    while (!open.empty() && !found)
    {
        ps_coord current = open.top();
        open.pop();
        if ((performance_met.cancel_flag && *performance_met.cancel_flag) || performance_met.timed_out()) break;
        performance_met.num_of_nodes_expanded++;

        if (current == goal)
        {
            found = true;
            break;
        }

        closed[current.second][current.first] = true;

        for (int d = 0; d < 8; d++)
        {
            int nx = current.first + dx[d];
            int ny = current.second + dy[d];

            if (nx < 0 || nx >= w || ny < 0 || ny >= h)
                continue;
            if (!is_traversable(map_data, nx, ny))
                continue;

            ps_coord neighbor = {nx, ny};
            performance_met.num_of_nodes_explored++;

            if (closed[neighbor.second][neighbor.first])
                continue;

            ps_coord parent = came_from[current.second][current.first];

            if (line_of_sight(parent, neighbor, map_data))
            {
                float tent_g = g_score[parent.second][parent.first] + euclidean_distance(parent, neighbor);

                if (tent_g < g_score[neighbor.second][neighbor.first])
                {
                    g_score[neighbor.second][neighbor.first] = tent_g;
                    came_from[neighbor.second][neighbor.first] = parent;
                    open.push(neighbor);
                    if (open.size() > performance_met.peak_open_size)
                        performance_met.peak_open_size = open.size();
                }
            }
            else
            {
                float tent_g = g_score[current.second][current.first] + euclidean_distance(current, neighbor);

                if (tent_g < g_score[neighbor.second][neighbor.first])
                {
                    g_score[neighbor.second][neighbor.first] = tent_g;
                    came_from[neighbor.second][neighbor.first] = current;
                    open.push(neighbor);
                    if (open.size() > performance_met.peak_open_size)
                        performance_met.peak_open_size = open.size();
                }
            }
        }
    }

    // Convert 2D vector came_from back to map (interface requirement)
    std::map<Coordinate, Coordinate> result;
    if (found)
    {
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++)
                if (came_from[y][x] != UNVISITED)
                    result[{x, y}] = came_from[y][x];
    }

    performance_met.success = found;
    auto end_time = std::chrono::high_resolution_clock::now();
    performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    return result;
}

} // namespace sapf
} // namespace solvers
} // namespace path_sync

#ifdef PATH_SYNC_BUILD_AS_PLUGIN
extern "C"
{

const char *plugin_name() { return "Theta_Star_Solver"; }
bool plugin_is_optimal() { return false; }
bool plugin_is_multi_agent() { return false; }
void *plugin_create() { return new path_sync::solvers::sapf::Theta_Star_Solver(); }
void plugin_destroy(void *p) { delete static_cast<path_sync::solvers::sapf::Theta_Star_Solver *>(p); }

}
#endif
