#include "path_sync/solvers/astar_solver.hpp"
#include "path_sync/grid.hpp"
#include "path_sync/psync_types.hpp"

#include <SFML/System/Vector2.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <queue>
#include <vector>

int getManhattandistance(Coordinate start, Coordinate end)
{
    return abs(start.first - end.first) + abs(start.second - end.second);
}


namespace psync
{
namespace solvers
{
namespace sapf
{



std::string Astar_Solver::get_solver_name() const
{
    return solver_name;
}

std::map<Coordinate, Coordinate> Astar_Solver::solve(psync::Grid &grid, Coordinate start, Coordinate end,
                                                     PerformanceMetrics &performance_met)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    // DECLARATION
    const int INF = 99999;
    Coordinate grid_size = grid.get_grid_size();
    bool visited[grid_size.first][grid_size.second];

    /* (distance from start node to node i: g-cost,
     * distance from end node to node i: f-cost) */
    std::vector<std::vector<Coordinate>> cost_mat;

    /* this lambdafn compares two cost_mat entries and prioratizes the one with least f-cost*/
    auto lambdafn = [](std::pair<Coordinate, Coordinate> left, std::pair<Coordinate, Coordinate> right) {
        return (left.second.first + left.second.second) > (right.second.first + right.second.second);
    };
    std::priority_queue<std::pair<Coordinate, Coordinate>, std::vector<std::pair<Coordinate, Coordinate>>,
                        decltype(lambdafn)>
        priority_queue(lambdafn); // <Position of node, (f-value, which is cost_mat)>

    bool reachedEnd = false;
    std::map<Coordinate, Coordinate> came_from;

    // INITIALIZATION

    for (int i = 0; i < grid_size.first; i++)
    {
        std::vector<Coordinate> temp;
        for (int j = 0; j < grid_size.second; j++)
        {
            temp.push_back(Coordinate(INF, INF));
        }
        cost_mat.push_back(temp);
    }

    cost_mat[start.second][start.first] = Coordinate(0, getManhattandistance(start, end));
    came_from[start] = Coordinate(-1, -1);
    priority_queue.push(std::pair<Coordinate, Coordinate>(start, cost_mat[start.second][start.first]));

    while (not priority_queue.empty() and not reachedEnd)
    {
        std::pair<Coordinate, Coordinate> visiting = priority_queue.top();
        priority_queue.pop();
        performance_met.num_of_nodes_expanded++;

        std::vector<Coordinate> neighbours = grid.find_neighbors_b2(visiting.first);
        for (Coordinate &n : neighbours)
        {
            performance_met.num_of_nodes_explored++;
            int new_dist = cost_mat[visiting.first.second][visiting.first.first].first + 1;
            int new_manhattan_dist = getManhattandistance(n, end);

            if (new_dist < cost_mat[n.second][n.first].first)
            {
                cost_mat[n.second][n.first].first = new_dist;
                cost_mat[n.second][n.first].second = new_manhattan_dist;

                priority_queue.push(std::pair<Coordinate, Coordinate>(n, Coordinate(new_dist, new_manhattan_dist)));

                came_from[n] = visiting.first;
            }

            if (n == end)
            {
                reachedEnd = true;
                // grid.getCell(n.first, n.second).setCellType(END);
                break;
            }
        }
    }

    if (not reachedEnd)
        came_from.clear();

    auto end_time = std::chrono::high_resolution_clock::now();
    performance_met.runtime = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    return came_from;
}

} // namespace sapf
} // namespace solvers
} // namespace psync

