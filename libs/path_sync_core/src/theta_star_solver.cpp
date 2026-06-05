#include "path_sync_core/solvers/theta_star_solver.hpp"

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
    return (dx + dy) + (std::max(dx, dy) - std::min(dx, dy));
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

    std::map<Coordinate, float> g_score;
    std::map<Coordinate, Coordinate> came_from;
    std::map<Coordinate, bool> closed;

    auto cmp = [&](ps_coord a, ps_coord b)
    {
        float fa = g_score[a] + octile_distance(a, goal);
        float fb = g_score[b] + octile_distance(b, goal);
        if (fa != fb) return fa > fb;
        return a < b;
    };
    std::priority_queue<ps_coord, std::vector<ps_coord>, decltype(cmp)> open(cmp);

    g_score[start] = 0;
    came_from[start] = start;
    open.push(start);
    performance_met.peak_open_size = open.size();

    const int dx[] = {0, 1, 1, 1, 0, -1, -1, -1};
    const int dy[] = {-1, -1, 0, 1, 1, 1, 0, -1};

    bool found = false;

    while (!open.empty() && !found)
    {
        ps_coord current = open.top();
        open.pop();
        if (performance_met.cancel_flag && *performance_met.cancel_flag) break;
        performance_met.num_of_nodes_expanded++;

        if (current == goal)
        {
            found = true;
            break;
        }

        closed[current] = true;

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

            if (closed[neighbor])
                continue;

            if (line_of_sight(came_from[current], neighbor, map_data))
            {
                ps_coord grandparent = came_from[current];
                float tent_g = g_score[grandparent] + euclidean_distance(grandparent, neighbor);

                if (!g_score.count(neighbor) || tent_g < g_score[neighbor])
                {
                    g_score[neighbor] = tent_g;
                    came_from[neighbor] = grandparent;
                    open.push(neighbor);
                    if (open.size() > performance_met.peak_open_size)
                        performance_met.peak_open_size = open.size();
                }
            }
            else
            {
                float tent_g = g_score[current] + euclidean_distance(current, neighbor);

                if (!g_score.count(neighbor) || tent_g < g_score[neighbor])
                {
                    g_score[neighbor] = tent_g;
                    came_from[neighbor] = current;
                    open.push(neighbor);
                    if (open.size() > performance_met.peak_open_size)
                        performance_met.peak_open_size = open.size();
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
