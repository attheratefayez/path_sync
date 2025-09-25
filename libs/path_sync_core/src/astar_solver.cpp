#include "path_sync_core/solvers/astar_solver.hpp"

#include <chrono>
#include <cmath>
#include <map>
#include <queue>
#include <vector>

#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/path_sync_types.hpp"

namespace
{
int getManhattanDistance(path_sync::Coordinate start, path_sync::Coordinate goal)
{
    return abs(start.first - goal.first) + abs(start.second - goal.second);
}

std::vector<path_sync::Coordinate> find_neighbors(const path_sync::MapData &map_data, path_sync::Coordinate candidate)
{
    const int x_moves[] = {0, 1, 0, -1};
    const int y_moves[] = {1, 0, -1, 0};
    std::vector<path_sync::Coordinate> neighbors;

    for (int i = 0; i < 4; i++)
    {
        int n_col = candidate.first + y_moves[i];
        int n_row = candidate.second + x_moves[i];

        if (map_data.get_cell_type(n_col, n_row) != path_sync::CellType::WALL)
        {
            neighbors.push_back({n_col, n_row});
        }
    }
    return neighbors;
}
} // namespace

namespace path_sync
{
namespace solvers
{
namespace sapf
{

std::string_view Astar_Solver::get_solver_name() const
{
    return solver_name_;
}

std::map<Coordinate, Coordinate> Astar_Solver::solve(const path_sync::MapData &map_data, Coordinate start,
                                                     Coordinate goal, PerformanceMetrics &performance_met)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    const int INF = 99999;
    const int width = map_data.get_width();
    const int height = map_data.get_height();

    std::vector<std::vector<Coordinate>> cost_mat(height, std::vector<Coordinate>(width, {INF, INF}));

    auto lambdafn = [](const std::pair<Coordinate, Coordinate> &left, const std::pair<Coordinate, Coordinate> &right) {
        return (left.second.first + left.second.second) > (right.second.first + right.second.second);
    };
    std::priority_queue<std::pair<Coordinate, Coordinate>, std::vector<std::pair<Coordinate, Coordinate>>,
                        decltype(lambdafn)>
        priority_queue(lambdafn);

    bool reachedEnd = false;
    std::map<Coordinate, Coordinate> came_from;

    cost_mat[start.second][start.first] = {0, getManhattanDistance(start, goal)};
    came_from[start] = {-1, -1};
    priority_queue.push({start, cost_mat[start.second][start.first]});

    while (!priority_queue.empty() && !reachedEnd)
    {
        std::pair<Coordinate, Coordinate> visiting = priority_queue.top();
        priority_queue.pop();
        performance_met.num_of_nodes_expanded++;

        std::vector<Coordinate> neighbors = find_neighbors(map_data, visiting.first);
        for (Coordinate &n : neighbors)
        {
            performance_met.num_of_nodes_explored++;
            int new_dist = cost_mat[visiting.first.second][visiting.first.first].first + 1;
            int new_manhattan_dist = getManhattanDistance(n, goal);

            if (new_dist < cost_mat[n.second][n.first].first)
            {
                cost_mat[n.second][n.first] = {new_dist, new_manhattan_dist};
                priority_queue.push({n, {new_dist, new_manhattan_dist}});
                came_from[n] = visiting.first;
            }

            if (n == goal)
            {
                reachedEnd = true;
                break;
            }
        }
    }

    if (!reachedEnd)
        came_from.clear();

    auto end_time = std::chrono::high_resolution_clock::now();
    performance_met.runtime = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    return came_from;
}

} // namespace sapf
} // namespace solvers
} // namespace path_sync
