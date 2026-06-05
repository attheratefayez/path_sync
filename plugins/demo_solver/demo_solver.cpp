#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <string_view>
#include <vector>

#include <path_sync_core/map_loader/map_data.hpp>
#include <path_sync_core/path_sync_types.hpp>
#include <path_sync_core/performance_mat.hpp>
#include <path_sync_core/solver_interface.hpp>

using path_sync::Coordinate;

class GreedyBestFirstSolver : public ISolver
{
public:
    std::string_view get_solver_name() const override
    {
        return "Greedy Best-First (Demo Plugin)";
    }

    bool is_optimal() const override { return false; }

    std::map<Coordinate, Coordinate> solve(
        const path_sync::MapData &map_data, Coordinate start, Coordinate goal,
        path_sync::PerformanceMetrics &performance_met) override
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        auto heuristic = [&](const Coordinate &a, const Coordinate &b) {
            return std::abs(static_cast<int>(a.first) - static_cast<int>(b.first))
                 + std::abs(static_cast<int>(a.second) - static_cast<int>(b.second));
        };

        using Node = std::pair<int, Coordinate>;
        std::priority_queue<Node, std::vector<Node>, std::greater<>> open;
        std::set<Coordinate> closed;
        std::map<Coordinate, Coordinate> came_from;
        std::map<Coordinate, int> g_score;

        open.emplace(heuristic(start, goal), start);
        g_score[start] = 0;
        came_from[start] = start;

        bool found = false;
        int explored = 0, expanded = 0, reopened = 0, peak_open = 1;

        while (!open.empty())
        {
            if ((performance_met.cancel_flag && *performance_met.cancel_flag) || performance_met.timed_out())
            {
                performance_met.success = false;
                performance_met.num_of_nodes_explored = explored;
                performance_met.num_of_nodes_expanded = expanded;
                performance_met.num_of_nodes_reopened = reopened;
                performance_met.peak_open_size = peak_open;
                auto end_time = std::chrono::high_resolution_clock::now();
                performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
                return {};
            }

            auto current = open.top().second;
            open.pop();
            expanded++;

            if (current == goal)
            {
                found = true;
                break;
            }

            if (closed.count(current))
                continue;
            closed.insert(current);

            int cx = static_cast<int>(current.first);
            int cy = static_cast<int>(current.second);
            int dirs[] = {-1, 0, 1, 0, -1};

            for (int d = 0; d < 4; ++d)
            {
                int nx = cx + dirs[d];
                int ny = cy + dirs[d + 1];
                if (nx < 0 || ny < 0) continue;
                auto uc = static_cast<std::size_t>(nx);
                auto vc = static_cast<std::size_t>(ny);
                if (uc >= map_data.get_width() || vc >= map_data.get_height()) continue;
                if (map_data.get_cell_type({uc, vc}) == path_sync::CellType::WALL) continue;

                Coordinate neighbor = {uc, vc};
                if (closed.count(neighbor)) continue;
                explored++;

                int tentative_g = g_score[current] + 1;
                if (!g_score.count(neighbor) || tentative_g < g_score[neighbor])
                {
                    g_score[neighbor] = tentative_g;
                    came_from[neighbor] = current;
                    open.emplace(heuristic(neighbor, goal), neighbor);
                    if (open.size() > static_cast<std::size_t>(peak_open))
                        peak_open = static_cast<int>(open.size());
                }
                else
                {
                    reopened++;
                }
            }
        }

        performance_met.success = found;
        performance_met.num_of_nodes_explored = explored;
        performance_met.num_of_nodes_expanded = expanded;
        performance_met.num_of_nodes_reopened = reopened;
        performance_met.peak_open_size = peak_open;
        auto end_time = std::chrono::high_resolution_clock::now();
        performance_met.runtime = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        if (!found)
            came_from.clear();

        return came_from;
    }
};

extern "C"
{

const char *plugin_name() { return "Greedy_Best_First"; }
bool plugin_is_optimal() { return false; }
bool plugin_is_multi_agent() { return false; }
void *plugin_create() { return new GreedyBestFirstSolver(); }
void plugin_destroy(void *p) { delete static_cast<GreedyBestFirstSolver *>(p); }

}
