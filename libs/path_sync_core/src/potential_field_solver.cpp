#include "path_sync_core/solvers/potential_field_solver.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include "path_sync_core/map_loader/cost_map.hpp"
#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/path_sync_types.hpp"

namespace
{

float dist(path_sync::Coordinate a, path_sync::Coordinate b)
{
    float dx = static_cast<float>(a.first - b.first);
    float dy = static_cast<float>(a.second - b.second);
    return std::sqrt(dx * dx + dy * dy);
}

bool is_free(const path_sync::MapData &map, int x, int y)
{
    return x >= 0 && x < map.get_width() && y >= 0 && y < map.get_height()
        && map.get_cell_type({x, y}) != path_sync::CellType::WALL;
}

} // anonymous namespace

namespace path_sync
{
namespace solvers
{
namespace sapf
{

PotentialFieldSolver::PotentialFieldSolver()
    : solver_name_("PotentialField")
    , weights_({1.0f, 1.0f, 1.0f, 1.0f, 1.0f})
{
}

std::string_view PotentialFieldSolver::get_solver_name() const
{
    return solver_name_;
}

std::map<Coordinate, Coordinate> PotentialFieldSolver::solve(
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
    int use_obj = have_cost ? std::min(num_objectives_, cost_map.objectives) : 0;

    const int w = map_data.get_width();
    const int h = map_data.get_height();

    float attr_gain = 1.0f;
    float rep_gain = 100.0f;

    std::map<Coordinate, Coordinate> came_from;
    Coordinate current = start;
    came_from[start] = {-1, -1};

    bool reached = false;
    for (int iter = 0; iter < MAX_ITER; iter++)
    {
        if ((performance_met.cancel_flag && *performance_met.cancel_flag)
            || performance_met.timed_out())
            break;
        performance_met.num_of_nodes_expanded++;

        if (current == goal)
        {
            reached = true;
            break;
        }

        float best_potential = 1e9f;
        Coordinate best_nbr = current;
        const int dx[] = {0, 1, 0, -1};
        const int dy[] = {1, 0, -1, 0};

        for (int i = 0; i < 4; i++)
        {
            int nx = current.first + dx[i];
            int ny = current.second + dy[i];
            if (!is_free(map_data, nx, ny))
                continue;
            performance_met.num_of_nodes_explored++;

            Coordinate nbr{nx, ny};

            float attractive = attr_gain * dist(nbr, goal);

            float repulsive = 0.0f;
            for (int ox = -2; ox <= 2; ox++)
            {
                for (int oy = -2; oy <= 2; oy++)
                {
                    if (!is_free(map_data, nx + ox, ny + oy))
                    {
                        float d = dist(nbr, {nx + ox, ny + oy});
                        if (d < 0.01f) d = 0.01f;
                        repulsive += rep_gain / (d * d);
                    }
                }
            }

            float obj_cost = 0.0f;
            if (have_cost)
            {
                for (int o = 0; o < use_obj; o++)
                {
                    float cv = cost_map.at(o, nx, ny);
                    if (cv < 0.0f) { obj_cost = 1e9f; break; }
                    obj_cost += weights_[o] * cv;
                }
            }

            float potential = attractive + repulsive + obj_cost;
            if (potential < best_potential)
            {
                best_potential = potential;
                best_nbr = nbr;
            }
        }

        if (best_nbr == current)
            break;

        came_from[best_nbr] = current;
        current = best_nbr;
    }

    if (reached)
    {
        performance_met.success = true;
        performance_met.path_length = came_from.size();
    }
    else
    {
        if (current == start)
            came_from.clear();
        performance_met.success = false;
    }

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
const char *plugin_name() { return "PotentialField"; }
bool plugin_is_optimal() { return false; }
bool plugin_is_multi_agent() { return false; }
void *plugin_create() { return new path_sync::solvers::sapf::PotentialFieldSolver(); }
void plugin_destroy(void *p) { delete static_cast<path_sync::solvers::sapf::PotentialFieldSolver *>(p); }
}
#endif
