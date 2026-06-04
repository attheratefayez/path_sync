#include "path_sync_core/solvers/epea_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
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

} // anonymous namespace

namespace path_sync
{
namespace solvers
{
namespace sapf
{

std::string_view EPEA_Star_Solver::get_solver_name() const
{
    return solver_name_;
}

std::map<Coordinate, Coordinate> EPEA_Star_Solver::solve(const path_sync::MapData &map_data, Coordinate start,
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

    const int ndx[] = {0, 1, 0, -1};
    const int ndy[] = {-1, 0, 1, 0};
    const int NUM_OP = 4;

    std::map<Coordinate, int> g_score;
    std::map<Coordinate, Coordinate> came_from;
    // Track how many operators have been applied for each node
    std::map<Coordinate, int> op_index;

    struct PQEntry
    {
        ps_coord pos;
        int f;
        int op_idx; // operator index to apply next time this node is expanded
    };

    auto cmp = [](const PQEntry &a, const PQEntry &b)
    {
        if (a.f != b.f) return a.f > b.f;
        if (a.op_idx != b.op_idx) return a.op_idx > b.op_idx;
        return a.pos < b.pos;
    };
    std::priority_queue<PQEntry, std::vector<PQEntry>, decltype(cmp)> open(cmp);

    g_score[start] = 0;
    came_from[start] = {-1, -1};
    op_index[start] = 0;
    open.push({start, manhattan_distance(start, goal), 0});

    bool found = false;

    while (!open.empty() && !found)
    {
        PQEntry entry = open.top();
        open.pop();
        ps_coord current = entry.pos;

        if (current == goal)
        {
            found = true;
            break;
        }

        performance_met.num_of_nodes_expanded++;

        // Get the next operator to apply
        int &cur_op_idx = op_index[current];

        // If this is a re-expansion, we need to use the stored op_idx from the queue entry
        if (entry.op_idx > 0)
        {
            cur_op_idx = entry.op_idx;
        }

        // Generate the successor for the current operator
        if (cur_op_idx < NUM_OP)
        {
            int nx = current.first + ndx[cur_op_idx];
            int ny = current.second + ndy[cur_op_idx];

            bool valid = (nx >= 0 && nx < w && ny >= 0 && ny < h &&
                         is_traversable(map_data, nx, ny));

            if (valid)
            {
                ps_coord neighbor = {nx, ny};
                int tent_g = g_score[current] + 1;
                performance_met.num_of_nodes_explored++;

                if (!g_score.count(neighbor) || tent_g < g_score[neighbor])
                {
                    g_score[neighbor] = tent_g;
                    came_from[neighbor] = current;

                    if (op_index.find(neighbor) == op_index.end())
                        op_index[neighbor] = 0;

                    open.push({neighbor, tent_g + manhattan_distance(neighbor, goal), 0});
                }
            }

            // Move to next operator
            cur_op_idx++;

            // If more operators remain, re-insert current node
            if (cur_op_idx < NUM_OP)
            {
                int f_val = g_score[current] + manhattan_distance(current, goal);
                open.push({current, f_val, cur_op_idx});
            }
        }
    }

    if (!found)
        came_from.clear();

    auto end_time = std::chrono::high_resolution_clock::now();
    performance_met.runtime = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    return came_from;
}

} // namespace sapf
} // namespace solvers
} // namespace path_sync
