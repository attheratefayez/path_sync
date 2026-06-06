#include "path_sync_core/solvers/pareto_rrt_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

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

std::vector<path_sync::Coordinate> free_neighbors(
    const path_sync::MapData &md, path_sync::Coordinate c)
{
    const int dx[] = {0, 1, 0, -1};
    const int dy[] = {1, 0, -1, 0};
    std::vector<path_sync::Coordinate> nbrs;
    for (int i = 0; i < 4; i++)
    {
        int nx = c.first + dx[i];
        int ny = c.second + dy[i];
        if (md.get_cell_type({nx, ny}) != path_sync::CellType::WALL)
            nbrs.push_back({nx, ny});
    }
    return nbrs;
}

bool is_free(const path_sync::MapData &md, int x, int y)
{
    return x >= 0 && x < md.get_width() && y >= 0 && y < md.get_height()
        && md.get_cell_type({x, y}) != path_sync::CellType::WALL;
}

std::mt19937 &rng()
{
    static std::mt19937 gen(std::random_device{}());
    return gen;
}

} // anonymous namespace

namespace path_sync
{
namespace solvers
{
namespace mo
{

std::vector<float> ParetoRRTSolver::node_cost(
    const CostMap *cost_map, const MapData &map_data,
    const std::vector<Coordinate> &path, int num_obj)
{
    std::vector<float> c(num_obj, 0.0f);
    int use_obj = cost_map ? std::min(num_obj, cost_map->objectives) : 0;

    for (std::size_t i = 0; i + 1 < path.size(); i++)
    {
        int x0 = path[i].first, y0 = path[i].second;
        int x1 = path[i + 1].first, y1 = path[i + 1].second;
        int steps = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
        if (steps == 0) steps = 1;
        for (int s = 0; s < steps; s++)
        {
            float t = (s + 1.0f) / steps;
            int cx = static_cast<int>(x0 + (x1 - x0) * t + 0.5f);
            int cy = static_cast<int>(y0 + (y1 - y0) * t + 0.5f);
            c[0] += 1.0f;
            if (use_obj > 0 && cost_map)
            {
                for (int o = 1; o < use_obj; o++)
                {
                    float cv = cost_map->at(o, cx, cy);
                    c[o] += (cv < 0 ? 1.0f : cv);
                }
            }
        }
    }
    return c;
}

std::optional<std::vector<MOSolution>> ParetoRRTSolver::solve(
    const MapData &map_data,
    const CostMap *cost_map,
    Coordinate start, Coordinate goal,
    int num_objectives,
    PerformanceMetrics &perf,
    MOMetrics &mo_met)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<RRTNode> nodes;
    nodes.push_back({start, {start}, std::vector<float>(num_objectives, 0.0f), -1});

    struct Solution
    {
        std::vector<Coordinate> path;
        std::vector<float> costs;
    };
    std::vector<Solution> solutions;

    const float step_size = 5.0f;
    int w = map_data.get_width();
    int h = map_data.get_height();

    for (int iter = 0; iter < MAX_ITER; iter++)
    {
        if ((perf.cancel_flag && *perf.cancel_flag) || perf.timed_out())
            break;
        perf.num_of_nodes_expanded++;

        // Sample random point (bias toward goal 20%)
        Coordinate sample;
        if (std::uniform_real_distribution<float>(0, 1)(rng()) < 0.2f)
        {
            sample = goal;
        }
        else
        {
            sample = {
                std::uniform_int_distribution<int>(0, w - 1)(rng()),
                std::uniform_int_distribution<int>(0, h - 1)(rng())
            };
            if (!is_free(map_data, sample.first, sample.second))
                continue;
        }

        // Find nearest node
        int nearest = 0;
        float min_d = 1e9f;
        for (int i = 0; i < static_cast<int>(nodes.size()); i++)
        {
            float d = dist(nodes[i].pos, sample);
            if (d < min_d) { min_d = d; nearest = i; }
        }

        // Steer toward sample
        auto &n = nodes[nearest];
        float dx = static_cast<float>(sample.first - n.pos.first);
        float dy = static_cast<float>(sample.second - n.pos.second);
        float d = std::sqrt(dx * dx + dy * dy);
        if (d < 0.1f) continue;

        int nx = n.pos.first + static_cast<int>(dx / d * step_size + 0.5f);
        int ny = n.pos.second + static_cast<int>(dy / d * step_size + 0.5f);
        nx = std::clamp(nx, 0, w - 1);
        ny = std::clamp(ny, 0, h - 1);

        if (!is_free(map_data, nx, ny))
            continue;

        // Check line of free cells
        bool blocked = false;
        int steps = std::max(std::abs(nx - n.pos.first), std::abs(ny - n.pos.second));
        for (int s = 1; s <= steps; s++)
        {
            float t = static_cast<float>(s) / steps;
            int cx = static_cast<int>(n.pos.first + (nx - n.pos.first) * t + 0.5f);
            int cy = static_cast<int>(n.pos.second + (ny - n.pos.second) * t + 0.5f);
            if (!is_free(map_data, cx, cy)) { blocked = true; break; }
        }
        if (blocked) continue;

        perf.num_of_nodes_explored++;

        // Build path to new node
        auto new_path = n.path;
        new_path.push_back({nx, ny});
        auto new_costs = node_cost(cost_map, map_data, new_path, num_objectives);

        nodes.push_back({{nx, ny}, new_path, new_costs, nearest});

        // Check if reached goal
        if (dist({nx, ny}, goal) <= step_size)
        {
            auto goal_path = new_path;
            goal_path.push_back(goal);
            auto goal_costs = node_cost(cost_map, map_data, goal_path, num_objectives);

            Solution sol{goal_path, goal_costs};

            // Check for Pareto dominance against existing solutions
            bool dominated = false;
            for (auto &existing : solutions)
            {
                bool ex_better = true;
                bool sol_better = true;
                for (int o = 0; o < num_objectives; o++)
                {
                    if (existing.costs[o] > sol.costs[o] + 1e-8f) ex_better = false;
                    if (sol.costs[o] > existing.costs[o] + 1e-8f) sol_better = false;
                }
                if (ex_better)
                {
                    dominated = true;
                    break;
                }
            }

            if (!dominated)
            {
                // Remove dominated existing solutions
                solutions.erase(
                    std::remove_if(solutions.begin(), solutions.end(),
                                   [&](const Solution &s) {
                                       bool better = true;
                                       for (int o = 0; o < num_objectives; o++)
                                           if (sol.costs[o] > s.costs[o] + 1e-8f)
                                               better = false;
                                       return better;
                                   }),
                    solutions.end());
                solutions.push_back(std::move(sol));
            }
        }
    }

    perf.success = !solutions.empty();

    std::vector<MOSolution> front;
    for (auto &s : solutions)
    {
        MOSolution ms;
        ms.path = s.path;
        ms.costs = s.costs;
        front.push_back(std::move(ms));
    }

    if (!front.empty())
    {
        float max_dist = 0;
        for (auto &s : front)
            if (s.costs[0] > max_dist) max_dist = s.costs[0];

        mo_met.front_size = static_cast<int>(front.size());
        mo_met.front = front;
        mo_met.ref_point = {max_dist * 1.1f, 1.1f, 1.1f, 1.1f, 1.1f};

        double hv = 0;
        for (auto &s : front)
        {
            double vol = 1.0;
            for (int i = 0; i < num_objectives; i++)
                vol *= (mo_met.ref_point[i] - s.costs[i]);
            hv += vol;
        }
        mo_met.hypervolume = hv / front.size();
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    perf.runtime = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time);

    return front.empty() ? std::nullopt : std::make_optional(std::move(front));
}

} // namespace mo
} // namespace solvers
} // namespace path_sync

#ifdef PATH_SYNC_BUILD_AS_PLUGIN
extern "C"
{
const char *plugin_name() { return "ParetoRRT"; }
bool plugin_is_optimal() { return false; }
bool plugin_is_multi_agent() { return false; }
bool plugin_is_mo() { return true; }
void *plugin_create() { return new path_sync::solvers::mo::ParetoRRTSolver(); }
void plugin_destroy(void *p) { delete static_cast<path_sync::solvers::mo::ParetoRRTSolver *>(p); }
}
#endif
