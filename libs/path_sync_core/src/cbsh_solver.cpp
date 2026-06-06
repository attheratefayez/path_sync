#include "path_sync_core/solvers/cbs_solver.hpp"

#include <algorithm>
#include <vector>

namespace path_sync::solvers::mapf
{

// ── CBSH: CG heuristic via minimum vertex cover ─────────────────────────

int CBS_Solver::compute_cg_heuristic(
    const std::vector<std::vector<Coordinate>> &paths) const
{
    if (paths.size() <= 1)
        return 0;

    int n = static_cast<int>(paths.size());

    // Build conflict graph
    std::vector<std::vector<bool>> graph(n, std::vector<bool>(n, false));

    size_t max_len = 0;
    for (const auto &p : paths)
        max_len = std::max(max_len, p.size());

    for (size_t t = 0; t < max_len; t++)
    {
        for (int a = 0; a < n; a++)
        {
            for (int b = a + 1; b < n; b++)
            {
                Coordinate pa = (t < paths[a].size()) ? paths[a][t] : paths[a].back();
                Coordinate pb = (t < paths[b].size()) ? paths[b][t] : paths[b].back();

                if (pa == pb)
                    graph[a][b] = graph[b][a] = true;

                if (t > 0)
                {
                    Coordinate pa_prev = paths[a][t - 1];
                    Coordinate pb_prev = paths[b][t - 1];
                    if (pa == pb_prev && pb == pa_prev)
                        graph[a][b] = graph[b][a] = true;
                }
            }
        }
    }

    return min_vertex_cover_size(graph);
}

int CBS_Solver::min_vertex_cover_size(
    const std::vector<std::vector<bool>> &graph) const
{
    int n = static_cast<int>(graph.size());

    // Check if vertex set (bitmask) is a vertex cover
    auto is_cover = [&](int mask) -> bool {
        for (int i = 0; i < n; i++)
        {
            if (mask & (1 << i)) continue;
            for (int j = i + 1; j < n; j++)
            {
                if (mask & (1 << j)) continue;
                if (graph[i][j])
                    return false;
            }
        }
        return true;
    };

    int best = n;
    int total = 1 << n;
    for (int mask = 0; mask < total; mask++)
    {
        int cnt = __builtin_popcount(mask);
        if (cnt >= best) continue;
        if (is_cover(mask))
            best = cnt;
    }

    return best;
}

} // namespace path_sync::solvers::mapf
