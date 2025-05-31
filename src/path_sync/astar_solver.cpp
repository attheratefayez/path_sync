#include <SFML/System/Vector2.hpp>
#include <chrono>
#include <cmath>
#include <map>
#include <queue>
#include <vector>

#include "path_sync/path_sync_types.hpp"
#include "path_sync/solvers/astar_solver.hpp"
#include "path_sync/visualization_system/grid.hpp"


int getManhattandistance(Coordinate start, Coordinate goal)
{
    return abs(start.first - goal.first) + abs(start.second - goal.second);
}

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

SolverType Astar_Solver::get_solver_type() const 
{
    return solver_type_;
}
std::map<Coordinate, Coordinate> Astar_Solver::solve(path_sync::Grid &grid, Coordinate start, Coordinate goal,
                                                     PerformanceMetrics &performance_met)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    // DECLARATION
    const int INF = 99999;
    Coordinate grid_size = grid.get_grid_size();
    bool visited[grid_size.first][grid_size.second];

    /* (distance from start node to node i: g-cost,
     * distance from goal node to node i: f-cost) */
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

    cost_mat[start.second][start.first] = Coordinate(0, getManhattandistance(start, goal));
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
            int new_manhattan_dist = getManhattandistance(n, goal);

            if (new_dist < cost_mat[n.second][n.first].first)
            {
                cost_mat[n.second][n.first].first = new_dist;
                cost_mat[n.second][n.first].second = new_manhattan_dist;

                priority_queue.push(std::pair<Coordinate, Coordinate>(n, Coordinate(new_dist, new_manhattan_dist)));

                came_from[n] = visiting.first;
            }

            if (n == goal)
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
} // namespace path_sync
