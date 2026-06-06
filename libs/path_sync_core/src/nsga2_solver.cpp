#include "path_sync_core/solvers/nsga2_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <vector>

#include "path_sync_core/map_loader/cost_map.hpp"
#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/path_sync_types.hpp"

namespace
{

int manhattan(path_sync::Coordinate a, path_sync::Coordinate b)
{
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
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

void NSGA2Solver::evaluate(Individual &ind, const MapData &map_data,
                            const CostMap *cost_map, Coordinate start,
                            Coordinate goal, int num_obj)
{
    ind.costs.assign(num_obj, 0.0f);
    int use_obj = cost_map ? std::min(num_obj, cost_map->objectives) : 0;

    // Build full path from start → waypoints → goal
    std::vector<Coordinate> full;
    full.push_back(start);
    for (auto &w : ind.path)
        full.push_back(w);
    full.push_back(goal);

    // Validate: replace blocked waypoints
    for (auto &wp : ind.path)
    {
        if (map_data.get_cell_type(wp) == CellType::WALL)
        {
            // Random walk from previous valid position to find replacement
            Coordinate prev = full.size() >= 2 ? full[full.size() - 2] : start;
            for (int attempt = 0; attempt < 20; attempt++)
            {
                auto nbrs = free_neighbors(map_data, prev);
                if (nbrs.empty()) break;
                wp = nbrs[std::uniform_int_distribution<int>(0, static_cast<int>(nbrs.size()) - 1)(rng())];
                if (map_data.get_cell_type(wp) != CellType::WALL)
                    break;
            }
        }
    }

    bool reaches_goal = false;
    if (!ind.path.empty())
    {
        Coordinate last = ind.path.back();
        reaches_goal = (last.first == goal.first && last.second == goal.second);
    }

    // Compute costs along path segments
    for (std::size_t i = 0; i + 1 < full.size(); i++)
    {
        int x0 = full[i].first, y0 = full[i].second;
        int x1 = full[i + 1].first, y1 = full[i + 1].second;
        int steps = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
        if (steps == 0) steps = 1;
        for (int s = 0; s < steps; s++)
        {
            float t = (s + 1.0f) / steps;
            int cx = static_cast<int>(x0 + (x1 - x0) * t + 0.5f);
            int cy = static_cast<int>(y0 + (y1 - y0) * t + 0.5f);
            ind.costs[0] += 1.0f; // distance
            if (use_obj > 0 && cost_map)
            {
                for (int o = 1; o < use_obj; o++)
                {
                    float cv = cost_map->at(o, cx, cy);
                    ind.costs[o] += (cv < 0 ? 1.0f : cv);
                }
            }
        }
    }

    // Penalize paths that don't reach the goal
    if (!reaches_goal)
    {
        float penalty = 1e6f;
        for (int o = 0; o < num_obj; o++)
            ind.costs[o] += penalty;
    }
}

void NSGA2Solver::fast_non_dominated_sort(std::vector<Individual> &pop, int num_obj)
{
    for (auto &ind : pop)
        ind.rank = 0;

    std::vector<std::vector<int>> dominated(pop.size());
    std::vector<int> domination_count(pop.size(), 0);
    std::vector<std::vector<int>> fronts;
    fronts.push_back({});

    for (std::size_t i = 0; i < pop.size(); i++)
    {
        for (std::size_t j = i + 1; j < pop.size(); j++)
        {
            bool i_dom_j = true, j_dom_i = true;
            for (int o = 0; o < num_obj; o++)
            {
                if (pop[i].costs[o] > pop[j].costs[o]) i_dom_j = false;
                if (pop[j].costs[o] > pop[i].costs[o]) j_dom_i = false;
            }
            if (i_dom_j && i != j)
            {
                dominated[i].push_back(static_cast<int>(j));
                domination_count[j]++;
            }
            else if (j_dom_i && i != j)
            {
                dominated[j].push_back(static_cast<int>(i));
                domination_count[i]++;
            }
        }
        if (domination_count[i] == 0)
        {
            pop[i].rank = 1;
            fronts[0].push_back(static_cast<int>(i));
        }
    }

    int f = 0;
    while (!fronts[f].empty())
    {
        std::vector<int> next;
        for (int i : fronts[f])
        {
            for (int j : dominated[i])
            {
                domination_count[j]--;
                if (domination_count[j] == 0)
                {
                    pop[j].rank = f + 2;
                    next.push_back(j);
                }
            }
        }
        f++;
        fronts.push_back(next);
    }
}

void NSGA2Solver::crowding_distance(std::vector<Individual> &pop, int num_obj)
{
    for (auto &ind : pop)
        ind.crowding = 0.0f;

    if (pop.size() <= 2) return;

    for (int o = 0; o < num_obj; o++)
    {
        std::sort(pop.begin(), pop.end(),
                  [o](const Individual &a, const Individual &b) {
                      return a.costs[o] < b.costs[o];
                  });

        float min_c = pop.front().costs[o];
        float max_c = pop.back().costs[o];
        float range = max_c - min_c;
        if (range < 1e-8f) range = 1e-8f;

        pop.front().crowding = 1e9f;
        pop.back().crowding = 1e9f;

        for (std::size_t i = 1; i + 1 < pop.size(); i++)
            pop[i].crowding += (pop[i + 1].costs[o] - pop[i - 1].costs[o]) / range;
    }
}

std::vector<Coordinate> NSGA2Solver::random_path(
    const MapData &map_data, Coordinate start, Coordinate goal)
{
    std::vector<Coordinate> path;
    Coordinate cur = start;
    int max_len = 2000;
    int stuck = 0;

    while (cur != goal && static_cast<int>(path.size()) < max_len)
    {
        auto nbrs = free_neighbors(map_data, cur);
        if (nbrs.empty()) break;

        // Bias toward goal
        std::vector<float> scores;
        float total = 0;
        for (auto &n : nbrs)
        {
            float s = 1.0f / (1.0f + manhattan(n, goal));
            scores.push_back(s);
            total += s;
        }

        float r = std::uniform_real_distribution<float>(0, total)(rng());
        float accum = 0;
        Coordinate chosen = nbrs.back();
        for (std::size_t i = 0; i < nbrs.size(); i++)
        {
            accum += scores[i];
            if (r <= accum) { chosen = nbrs[i]; break; }
        }

        if (chosen == cur) break;

        // Allow revisiting nodes — prevent getting stuck on small maps
        if (std::find(path.begin(), path.end(), chosen) != path.end())
        {
            stuck++;
            if (stuck > 200) { path.clear(); break; }
            continue;
        }
        stuck = 0;
        path.push_back(chosen);
        cur = chosen;
    }

    // Retry once if failed
    if (path.empty() || path.back() != goal)
    {
        path.clear();
        cur = start;
        stuck = 0;
        while (cur != goal && static_cast<int>(path.size()) < max_len)
        {
            auto nbrs = free_neighbors(map_data, cur);
            if (nbrs.empty()) break;
            float r = std::uniform_real_distribution<float>(0, 1)(rng());
            Coordinate chosen = nbrs[static_cast<int>(r * nbrs.size())];
            if (chosen == cur) break;
            if (std::find(path.begin(), path.end(), chosen) != path.end())
            {
                stuck++;
                if (stuck > 200) break;
                continue;
            }
            stuck = 0;
            path.push_back(chosen);
            cur = chosen;
        }
    }

    return path;
}

std::vector<Coordinate> NSGA2Solver::crossover(
    const std::vector<Coordinate> &p1, const std::vector<Coordinate> &p2)
{
    if (p1.empty() || p2.empty()) return p1.empty() ? p2 : p1;

    // Find common waypoints
    std::set<Coordinate> p2_set(p2.begin(), p2.end());
    std::vector<int> common_idx;
    for (std::size_t i = 0; i < p1.size(); i++)
        if (p2_set.count(p1[i])) common_idx.push_back(static_cast<int>(i));

    std::vector<Coordinate> child;
    if (common_idx.size() >= 2)
    {
        int a = common_idx[std::uniform_int_distribution<int>(
            0, static_cast<int>(common_idx.size()) - 1)(rng())];
        int b = common_idx[std::uniform_int_distribution<int>(
            0, static_cast<int>(common_idx.size()) - 1)(rng())];
        if (a > b) std::swap(a, b);

        child.insert(child.end(), p1.begin(), p1.begin() + a);
        child.insert(child.end(), p2.begin() + a, p2.begin() + b);
        child.insert(child.end(), p1.begin() + b, p1.end());
    }
    else
    {
        int split = std::uniform_int_distribution<int>(
            1, static_cast<int>(std::min(p1.size(), p2.size())) - 1)(rng());
        child.insert(child.end(), p1.begin(), p1.begin() + split);
        child.insert(child.end(), p2.begin() + split, p2.end());
    }

    return child;
}

void NSGA2Solver::mutate(std::vector<Coordinate> &path, const MapData &map_data)
{
    if (path.empty()) return;

    std::uniform_real_distribution<float> prob(0, 1);
    for (auto &wp : path)
    {
        if (prob(rng()) < 0.1f)
        {
            auto nbrs = free_neighbors(map_data, wp);
            if (!nbrs.empty())
                wp = nbrs[std::uniform_int_distribution<int>(
                    0, static_cast<int>(nbrs.size()) - 1)(rng())];
        }
    }
}

std::optional<std::vector<MOSolution>> NSGA2Solver::solve(
    const MapData &map_data,
    const CostMap *cost_map,
    Coordinate start, Coordinate goal,
    int num_objectives,
    PerformanceMetrics &perf,
    MOMetrics &mo_met)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    int ps = pop_size_;
    int mg = max_gen_;

    // Initialize population
    std::vector<Individual> pop(ps);
    for (int i = 0; i < ps; i++)
    {
        pop[i].path = random_path(map_data, start, goal);
        evaluate(pop[i], map_data, cost_map, start, goal, num_objectives);
    }

    for (int gen = 0; gen < mg; gen++)
    {
        if ((perf.cancel_flag && *perf.cancel_flag) || perf.timed_out())
            break;

        // Create offspring
        std::vector<Individual> offspring(ps);
        for (int i = 0; i < ps; i += 2)
        {
            int a = std::uniform_int_distribution<int>(0, ps - 1)(rng());
            int b = std::uniform_int_distribution<int>(0, ps - 1)(rng());

            auto child_path = crossover(pop[a].path, pop[b].path);
            mutate(child_path, map_data);

            offspring[i].path = child_path;
            evaluate(offspring[i], map_data, cost_map, start, goal, num_objectives);

            if (i + 1 < ps)
            {
                child_path = crossover(pop[b].path, pop[a].path);
                mutate(child_path, map_data);
                offspring[i + 1].path = child_path;
                evaluate(offspring[i + 1], map_data, cost_map, start, goal, num_objectives);
            }
        }

        // Combine and select
        std::vector<Individual> combined = pop;
        combined.insert(combined.end(), offspring.begin(), offspring.end());

        fast_non_dominated_sort(combined, num_objectives);

        std::vector<std::vector<Individual>> fronts;
        int max_rank = 0;
        for (auto &ind : combined)
            if (ind.rank > max_rank) max_rank = ind.rank;

        fronts.resize(max_rank);
        for (auto &ind : combined)
            fronts[ind.rank - 1].push_back(ind);

        pop.clear();
        for (auto &f : fronts)
        {
            if (static_cast<int>(pop.size() + f.size()) <= ps)
            {
                pop.insert(pop.end(), f.begin(), f.end());
            }
            else
            {
                crowding_distance(f, num_objectives);
                std::sort(f.begin(), f.end(),
                          [](const Individual &a, const Individual &b) {
                              return a.crowding > b.crowding;
                          });
                int need = ps - static_cast<int>(pop.size());
                pop.insert(pop.end(), f.begin(), f.begin() + need);
                break;
            }
        }

        perf.num_of_nodes_expanded += ps;
        perf.num_of_nodes_explored += ps * 10;
    }

    // Extract Pareto front from rank-1 individuals
    fast_non_dominated_sort(pop, num_objectives);
    std::vector<MOSolution> front;
    for (auto &ind : pop)
    {
        if (ind.rank != 1) continue;

        // Skip penalized individuals (costs[0] >= 1e5 marks infeasible)
        if (!ind.costs.empty() && ind.costs[0] >= 1e5f) continue;

        MOSolution sol;
        sol.costs = ind.costs;
        sol.crowding_distance = ind.crowding;

        // Build full path from start → waypoints
        sol.path.push_back(start);
        sol.path.insert(sol.path.end(), ind.path.begin(), ind.path.end());

        front.push_back(std::move(sol));
    }

    // Final Pareto sort on front
    std::vector<MOSolution> pareto;
    for (auto &sol : front)
    {
        bool dominated = false;
        for (auto &other : front)
        {
            if (&sol != &other && other.dominates(sol, num_objectives))
            { dominated = true; break; }
        }
        if (!dominated)
            pareto.push_back(sol);
    }

    perf.success = !pareto.empty();

    if (!pareto.empty())
    {
        float max_dist = 0;
        for (auto &s : pareto)
            if (s.costs[0] > max_dist) max_dist = s.costs[0];

        mo_met.front_size = static_cast<int>(pareto.size());
        mo_met.front = pareto;
        mo_met.ref_point = {max_dist * 1.1f, 1.1f, 1.1f, 1.1f, 1.1f};

        double hv = 0;
        for (auto &s : pareto)
        {
            double vol = 1.0;
            for (int i = 0; i < num_objectives; i++)
                vol *= (mo_met.ref_point[i] - s.costs[i]);
            hv += vol;
        }
        mo_met.hypervolume = hv / pareto.size();
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    perf.runtime = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time);

    return pareto.empty() ? std::nullopt : std::make_optional(std::move(pareto));
}

} // namespace mo
} // namespace solvers
} // namespace path_sync

#ifdef PATH_SYNC_BUILD_AS_PLUGIN
extern "C"
{
const char *plugin_name() { return "NSGA2"; }
bool plugin_is_optimal() { return false; }
bool plugin_is_multi_agent() { return false; }
bool plugin_is_mo() { return true; }
void *plugin_create() { return new path_sync::solvers::mo::NSGA2Solver(); }
void plugin_destroy(void *p) { delete static_cast<path_sync::solvers::mo::NSGA2Solver *>(p); }
}
#endif
