#include <chrono>
#include <deque>
#include "path_sync/bfs_solver.hpp"
#include "path_sync/psync_types.hpp"

namespace psync {

std::string BFS_Solver::get_solver_name() const
{
    return solver_name;
}

std::map<Coordinate, Coordinate> BFS_Solver::solve(Grid &grid, Coordinate start, Coordinate end, PerformanceMetrics& performance_met)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    // DECLARATION
    bool reachedEnd = false;
    Coordinate grid_size = grid.get_grid_size();

    bool visited[grid_size.first][grid_size.second];
    std::map<Coordinate, Coordinate> came_from;
    std::deque<Coordinate> queue;

    // INITIALIZATION
    for (int i = 0; i < grid_size.first; i++)
        for (int j = 0; j < grid_size.second; j++)
            visited[i][j] = false;

    came_from.insert(std::pair<Coordinate, Coordinate>(start, Coordinate(-1, -1)));
    queue.push_back(start);

    while (not queue.empty() and not reachedEnd)
    {
        Coordinate visiting = queue.front();
        queue.pop_front();
        performance_met.num_of_nodes_expanded++;

        if (visited[visiting.second][visiting.first])
            continue;
        visited[visiting.second][visiting.first] = true;

        std::vector<Coordinate> neighbours = grid.find_neighbors_b2(visiting);
        for (Coordinate &n : neighbours)
        {
            if (not visited[n.second][n.first] and std::find(queue.begin(), queue.end(), n) == queue.end())
            {
                performance_met.num_of_nodes_explored++;
                queue.push_back(n);
                came_from[n] = visiting;

                if (n == end)
                {
                    reachedEnd = true;
                    break;
                }
            }
        }
    }

    if (not reachedEnd)
        came_from.clear();

    auto end_time = std::chrono::high_resolution_clock::now();
    performance_met.runtime = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    return came_from;
}

}
