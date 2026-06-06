#include "path_sync_core/solvers/mo_astar_solver.hpp"

#include <algorithm>
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

struct Label
{
    path_sync::Coordinate pos;
    std::vector<float> g;
    std::vector<float> h;
    std::shared_ptr<Label> parent;

    float sum() const
    {
        float s = 0;
        for (auto v : g) s += v;
        for (auto v : h) s += v;
        return s;
    }
};

struct LabelCmp
{
    bool operator()(const std::shared_ptr<Label> &a,
                    const std::shared_ptr<Label> &b) const
    {
        return a->sum() > b->sum();
    }
};

bool dominates(const std::vector<float> &a, const std::vector<float> &b)
{
    bool better = false;
    for (std::size_t i = 0; i < a.size(); i++)
    {
        if (a[i] > b[i] + 1e-8f) return false;
        if (a[i] < b[i] - 1e-8f) better = true;
    }
    return better;
}

bool is_dominated_by_any(const std::vector<float> &v,
                          const std::vector<std::vector<float>> &set)
{
    for (auto &s : set)
        if (dominates(s, v))
            return true;
    return false;
}

float heuristic(path_sync::Coordinate a, path_sync::Coordinate b)
{
    return static_cast<float>(
        std::abs(a.first - b.first) + std::abs(a.second - b.second));
}

std::vector<path_sync::Coordinate> neighbors(
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

} // anonymous namespace

namespace path_sync
{
namespace solvers
{
namespace mo
{

std::optional<std::vector<MOSolution>> MOAStarSolver::solve(
    const MapData &map_data,
    const CostMap *cost_map,
    Coordinate start, Coordinate goal,
    int num_objectives,
    PerformanceMetrics &perf,
    MOMetrics &mo_met)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    int use_obj = cost_map ? std::min(num_objectives, cost_map->objectives) : 0;
    if (use_obj < 2) use_obj = num_objectives;

    using LabelSet = std::vector<std::vector<float>>;
    std::map<std::pair<int,int>, LabelSet> cell_labels;

    std::priority_queue<std::shared_ptr<Label>,
                        std::vector<std::shared_ptr<Label>>,
                        LabelCmp> pq;

    auto init_g = std::vector<float>(use_obj, 0.0f);
    auto init_h = std::vector<float>(use_obj, 0.0f);
    float h_base = heuristic(start, goal);
    for (int i = 0; i < use_obj; i++)
        init_h[i] = h_base;

    auto first = std::make_shared<Label>();
    first->pos = start;
    first->g = init_g;
    first->h = init_h;
    first->parent = nullptr;
    pq.push(first);

    std::vector<MOSolution> all_solutions;

    while (!pq.empty())
    {
        auto cur = pq.top();
        pq.pop();
        if ((perf.cancel_flag && *perf.cancel_flag) || perf.timed_out())
            break;
        perf.num_of_nodes_expanded++;

        auto key = std::make_pair(cur->pos.first, cur->pos.second);
        if (is_dominated_by_any(cur->g, cell_labels[key]))
            continue;
        cell_labels[key].push_back(cur->g);

        if (cur->pos == goal)
        {
            MOSolution sol;
            sol.costs = cur->g;
            auto l = cur;
            while (l)
            {
                sol.path.push_back(l->pos);
                l = l->parent;
            }
            std::reverse(sol.path.begin(), sol.path.end());

            bool dominated = false;
            for (auto &existing : all_solutions)
            {
                if (existing.dominates(sol, use_obj))
                { dominated = true; break; }
            }
            if (!dominated)
            {
                all_solutions.erase(
                    std::remove_if(all_solutions.begin(), all_solutions.end(),
                                   [&](const MOSolution &s) {
                                       return sol.dominates(s, use_obj);
                                   }),
                    all_solutions.end());
                all_solutions.push_back(std::move(sol));
            }
            continue;
        }

        auto nbrs = neighbors(map_data, cur->pos);
        for (auto &n : nbrs)
        {
            perf.num_of_nodes_explored++;

            float base = 1.0f;
            auto new_g = cur->g;
            if (cost_map)
            {
                for (int o = 0; o < use_obj; o++)
                {
                    float cv = cost_map->at(o, n.first, n.second);
                    if (cv < 0.0f) { new_g[o] = 1e9f; break; }
                    new_g[o] += cv;
                }
            }
            else
            {
                for (int o = 0; o < use_obj; o++)
                    new_g[o] += base;
            }

            auto nkey = std::make_pair(n.first, n.second);
            if (is_dominated_by_any(new_g, cell_labels[nkey]))
                continue;

            float h_val = heuristic(n, goal);
            auto new_h = std::vector<float>(use_obj, h_val);

            auto next = std::make_shared<Label>();
            next->pos = n;
            next->g = new_g;
            next->h = new_h;
            next->parent = cur;
            pq.push(next);
        }

        if (pq.size() > perf.peak_open_size)
            perf.peak_open_size = pq.size();
    }

    perf.success = !all_solutions.empty();

    if (!all_solutions.empty())
    {
        float max_dist = 0;
        for (auto &s : all_solutions)
            if (s.costs[0] > max_dist) max_dist = s.costs[0];

        mo_met.front_size = static_cast<int>(all_solutions.size());
        mo_met.front = all_solutions;
        mo_met.ref_point = {max_dist * 1.1f, 1.1f, 1.1f, 1.1f, 1.1f};

        double hv = 0;
        for (auto &s : all_solutions)
        {
            double vol = 1.0;
            for (int i = 0; i < use_obj; i++)
                vol *= (mo_met.ref_point[i] - s.costs[i]);
            hv += vol;
        }
        mo_met.hypervolume = hv / all_solutions.size();
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    perf.runtime = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time);

    if (all_solutions.empty())
        return std::nullopt;

    auto result = std::move(all_solutions);
    return result;
}

} // namespace mo
} // namespace solvers
} // namespace path_sync

#ifdef PATH_SYNC_BUILD_AS_PLUGIN
extern "C"
{
const char *plugin_name() { return "MOAStar (MO)"; }
bool plugin_is_optimal() { return true; }
bool plugin_is_multi_agent() { return false; }
bool plugin_is_mo() { return true; }
void *plugin_create() { return new path_sync::solvers::mo::MOAStarSolver(); }
void plugin_destroy(void *p) { delete static_cast<path_sync::solvers::mo::MOAStarSolver *>(p); }
}
#endif
