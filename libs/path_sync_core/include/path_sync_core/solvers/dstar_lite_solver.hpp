#ifndef __PATHFINDING_DSTAR_LITE_SOLVER_HPP__
#define __PATHFINDING_DSTAR_LITE_SOLVER_HPP__

#include "path_sync_core/performance_mat.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/solver_interface.hpp"

namespace path_sync
{
namespace solvers
{
namespace sapf
{

struct DStarKey
{
    float k0;
    float k1;

    bool operator<(const DStarKey &other) const noexcept
    {
        if (k0 != other.k0) return k0 < other.k0;
        return k1 < other.k1;
    }

    bool operator==(const DStarKey &other) const noexcept
    {
        return k0 == other.k0 && k1 == other.k1;
    }
};

class DStar_Lite_Solver : public ISolver
{
  private:
    const std::string solver_name_ = "DStar_Lite_Solver";

  public:
    std::string_view get_solver_name() const override;
    bool is_optimal() const override { return true; }

    std::map<Coordinate, Coordinate> solve(const path_sync::MapData &map_data, Coordinate start, Coordinate goal,
                                           PerformanceMetrics &performance_met) override;
};

} // namespace sapf
} // namespace solvers
} // namespace path_sync

#endif
