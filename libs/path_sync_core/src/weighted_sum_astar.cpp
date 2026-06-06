#include "path_sync_core/solvers/weighted_sum_astar.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <vector>

#include "path_sync_core/map_loader/cost_map.hpp"
#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/path_sync_types.hpp"

namespace
{

std::vector<path_sync::Coordinate> find_neighbors(
    const path_sync::MapData &map_data, path_sync::Coordinate c)
{
    const int dx[] = {0, 1, 0, -1};
    const int dy[] = {1, 0, -1, 0};
    std::vector<path_sync::Coordinate> nbrs;
    for (int i = 0; i < 4; i++)
    {
        int nx = c.first + dx[i];
        int ny = c.second + dy[i];
        if (map_data.get_cell_type({nx, ny}) != path_sync::CellType::WALL)
            nbrs.push_back({nx, ny});
    }
    return nbrs;
}

float get_heuristic(path_sync::Coordinate a, path_sync::Coordinate b)
{
    return static_cast<float>(std::abs(a.first - b.first) + std::abs(a.second - b.second));
}

} // anonymous namespace

namespace path_sync
{
namespace solvers
{
namespace sapf
{

WeightedSumAStar::WeightedSumAStar()
    : solver_name_("WeightedSumAStar")
    , weights_({1.0f, 1.0f, 1.0f, 1.0f, 1.0f})
{
}

std::string_view WeightedSumAStar::get_solver_name() const
{
    return solver_name_;
}

std::map<Coordinate, Coordinate> WeightedSumAStar::solve(
    const MapData &map_data, Coordinate start, Coordinate goal,
    PerformanceMetrics &performance_met)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    std::string map_name = map_data.get_map_info().map_name;
    std::string cost_path = std::string(PROJECT_ROOT) + "/maps/mo_costmaps/"
                          + map_name.substr(0, map_name.find_last_of('.'))
                          + ".cost";
    CostMap cost_map;
    bool have_cost = cost_map.load(cost_path);
    int use_obj = std::min(num_objectives_, have_cost ? cost_map.objectives : 0);

    const int w = map_data.get_width();
    const int h = map_data.get_height();

    struct CellCost { float g; float h; };
    std::vector<std::vector<CellCost>> cost_mat(h, std::vector<CellCost>(w, {1e9f, 1e9f}));

    auto cmp = [](const std::pair<Coordinate, CellCost> &a,
                   const std::pair<Coordinate, CellCost> &b) {
        return (a.second.g + a.second.h) > (b.second.g + b.second.h);
    };
    std::priority_queue<std::pair<Coordinate, CellCost>,
                        std::vector<std::pair<Coordinate, CellCost>>,
                        decltype(cmp)> pq(cmp);

    std::map<Coordinate, Coordinate> came_from;
    bool found = false;

    float h0 = get_heuristic(start, goal);
    cost_mat[start.second][start.first] = {0, h0};
    came_from[start] = {-1, -1};
    pq.push({start, {0, h0}});
    if (pq.size() > performance_met.peak_open_size)
        performance_met.peak_open_size = pq.size();

    while (!pq.empty() && !found)
    {
        auto cur = pq.top();
        pq.pop();
        if ((performance_met.cancel_flag && *performance_met.cancel_flag)
            || performance_met.timed_out())
            break;
        performance_met.num_of_nodes_expanded++;

        auto nbrs = find_neighbors(map_data, cur.first);
        for (auto &n : nbrs)
        {
            performance_met.num_of_nodes_explored++;

            float base_cost = 1.0f;
            float obj_cost = 0.0f;
            if (have_cost)
            {
                for (int o = 0; o < use_obj; o++)
                {
                    float cv = cost_map.at(o, n.first, n.second);
                    if (cv < 0.0f) { obj_cost = 1e9f; break; }
                    obj_cost += weights_[o] * cv;
                }
            }
            float step = base_cost + obj_cost;
            float new_g = cost_mat[cur.first.second][cur.first.first].g + step;

            if (new_g < cost_mat[n.second][n.first].g)
            {
                float nh = get_heuristic(n, goal);
                cost_mat[n.second][n.first] = {new_g, nh};
                pq.push({n, {new_g, nh}});
                if (pq.size() > performance_met.peak_open_size)
                    performance_met.peak_open_size = pq.size();
                came_from[n] = cur.first;
            }

            if (n == goal)
            {
                found = true;
                break;
            }
        }
    }

    if (!found)
        came_from.clear();

    performance_met.success = found;
    auto end_time = std::chrono::high_resolution_clock::now();
    performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time);

    return came_from;
}

} // namespace sapf
} // namespace solvers
} // namespace path_sync

#ifdef PATH_SYNC_BUILD_AS_PLUGIN
extern "C"
{
const char *plugin_name() { return "WeightedSumAStar (MO)"; }
bool plugin_is_optimal() { return false; }
bool plugin_is_multi_agent() { return false; }
void *plugin_create() { return new path_sync::solvers::sapf::WeightedSumAStar(); }
void plugin_destroy(void *p) { delete static_cast<path_sync::solvers::sapf::WeightedSumAStar *>(p); }
}
#endif
