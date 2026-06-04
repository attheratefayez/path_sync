#ifndef __PATHFINDING_EPEA_SOLVER_HPP__
#define __PATHFINDING_EPEA_SOLVER_HPP__

#include "path_sync_core/performance_mat.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/solver_interface.hpp"

namespace path_sync
{
namespace solvers
{
namespace sapf
{

class EPEA_Star_Solver : public ISolver
{
  private:
    const std::string solver_name_ = "EPEA_Star_Solver";

  public:
    std::string_view get_solver_name() const override;

    std::map<Coordinate, Coordinate> solve(const path_sync::MapData &map_data, Coordinate start, Coordinate goal,
                                           PerformanceMetrics &performance_met) override;
};

} // namespace sapf
} // namespace solvers
} // namespace path_sync

#endif
