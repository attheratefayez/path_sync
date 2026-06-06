#ifndef PATH_SYNC_PARETO_RRT_SOLVER_HPP
#define PATH_SYNC_PARETO_RRT_SOLVER_HPP

#include "path_sync_core/map_loader/cost_map.hpp"
#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/mo_types.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/performance_mat.hpp"
#include "path_sync_core/solvers/imo_solver.hpp"

#include <vector>

namespace path_sync
{
namespace solvers
{
namespace mo
{

class ParetoRRTSolver : public IMOSolver
{
public:
    std::string_view get_solver_name() const override { return "ParetoRRT"; }
    bool is_optimal() const override { return false; }
    int get_num_objectives() const override { return num_objectives_; }
    void set_num_objectives(int n) { num_objectives_ = n; }

    std::optional<std::vector<MOSolution>> solve(
        const MapData &map_data,
        const CostMap *cost_map,
        Coordinate start, Coordinate goal,
        int num_objectives,
        PerformanceMetrics &perf,
        MOMetrics &mo_met) override;

private:
    int num_objectives_ = 5;
    static constexpr int MAX_ITER = 10000;

    struct RRTNode
    {
        Coordinate pos;
        std::vector<Coordinate> path;
        std::vector<float> costs;
        int parent = -1;
    };

    std::vector<float> node_cost(const CostMap *cost_map,
                                  const MapData &map_data,
                                  const std::vector<Coordinate> &path,
                                  int num_obj);
};

} // namespace mo
} // namespace solvers
} // namespace path_sync

#endif // PATH_SYNC_PARETO_RRT_SOLVER_HPP
