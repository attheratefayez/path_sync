#ifndef PATH_SYNC_MO_ASTAR_SOLVER_HPP
#define PATH_SYNC_MO_ASTAR_SOLVER_HPP

#include "path_sync_core/map_loader/cost_map.hpp"
#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/mo_types.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/performance_mat.hpp"
#include "path_sync_core/solvers/imo_solver.hpp"

namespace path_sync
{
namespace solvers
{
namespace mo
{

class MOAStarSolver : public IMOSolver
{
public:
    std::string_view get_solver_name() const override { return "MOAStar"; }
    bool is_optimal() const override { return true; }
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
};

} // namespace mo
} // namespace solvers
} // namespace path_sync

#endif // PATH_SYNC_MO_ASTAR_SOLVER_HPP
