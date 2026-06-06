#ifndef PATH_SYNC_IMO_SOLVER_HPP
#define PATH_SYNC_IMO_SOLVER_HPP

#include <optional>
#include <string_view>
#include <vector>

#include "path_sync_core/map_loader/cost_map.hpp"
#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/mo_types.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/performance_mat.hpp"

namespace path_sync
{

class IMOSolver
{
public:
    virtual ~IMOSolver() = default;

    virtual std::string_view get_solver_name() const = 0;
    virtual bool is_optimal() const = 0;
    virtual int get_num_objectives() const { return 5; }
    virtual bool needs_weights() const { return false; }
    virtual void set_weights(const std::vector<float> &) {}

    virtual std::optional<std::vector<MOSolution>> solve(
        const MapData &map_data,
        const CostMap *cost_map,
        Coordinate start, Coordinate goal,
        int num_objectives,
        PerformanceMetrics &perf,
        MOMetrics &mo_met) = 0;
};

} // namespace path_sync

#endif // PATH_SYNC_IMO_SOLVER_HPP
