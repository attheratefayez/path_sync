#include "path_sync_core/solvers/bfs_solver.hpp"

#include <chrono>
#include <deque>
#include <vector>

#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/path_sync_types.hpp"

std::vector<path_sync::Coordinate> find_neighbors(const path_sync::MapData& map_data,
                                                  path_sync::Coordinate candidate) {
    const int x_moves[] = {0, 1, 0, -1};
    const int y_moves[] = {1, 0, -1, 0};
    std::vector<path_sync::Coordinate> neighbors;

    for (int i = 0; i < 4; i++) {
        int n_col = candidate.first + y_moves[i];
        int n_row = candidate.second + x_moves[i];

        if (map_data.get_cell_type(n_col, n_row) != path_sync::CellType::WALL) {
            neighbors.push_back({n_col, n_row});
        }
    }
    return neighbors;
}

namespace path_sync {
namespace solvers {
namespace sapf {

std::string_view BFS_Solver::get_solver_name() const {
    return solver_name_;
}

std::map<Coordinate, Coordinate> BFS_Solver::solve(const path_sync::MapData& map_data, Coordinate start,
                                                   Coordinate end, PerformanceMetrics& performance_met) {
    auto start_time = std::chrono::high_resolution_clock::now();
    bool reachedEnd = false;
    const int width = map_data.get_width();
    const int height = map_data.get_height();

    std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
    std::map<Coordinate, Coordinate> came_from;
    std::deque<Coordinate> queue;

    came_from[start] = {-1, -1};
    queue.push_back(start);

    while (!queue.empty() && !reachedEnd) {
        Coordinate visiting = queue.front();
        queue.pop_front();
        performance_met.num_of_nodes_expanded++;

        if (visited[visiting.second][visiting.first]) continue;
        visited[visiting.second][visiting.first] = true;

        std::vector<Coordinate> neighbors = find_neighbors(map_data, visiting);
        for (Coordinate& n : neighbors) {
            if (!visited[n.second][n.first]) {
                performance_met.num_of_nodes_explored++;
                queue.push_back(n);
                came_from[n] = visiting;

                if (n == end) {
                    reachedEnd = true;
                    break;
                }
            }
        }
    }

    if (!reachedEnd) came_from.clear();

    auto end_time = std::chrono::high_resolution_clock::now();
    performance_met.runtime = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    return came_from;
}

} // namespace sapf
} // namespace solvers
} // namespace path_sync
