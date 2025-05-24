#ifndef __PATHFINDING_BSFSOLVER_HPP__
#define __PATHFINDING_BSFSOLVER_HPP__

#include "path_sync/performance/performance_mat.hpp"
#include "path_sync/psync_types.hpp"
#include "path_sync/solver.hpp"
#include "path_sync/visualization_system/grid.hpp"

namespace psync
{
namespace solvers
{
namespace sapf
{

/**
 * @class BFS_Solver
 * @brief Solver class which uses BFS to get a solution.
 *
 */
class BFS_Solver : public ISolver
{
  private:
    const std::string solver_name = "BFS_Solver";

  public:
    std::string get_solver_name() const override;
    std::map<Coordinate, Coordinate> solve(Grid &grid, Coordinate start, Coordinate end, PerformanceMetrics& performance_met) override;
};


} // namespace sapf
} // namespace solvers
} // namespace psync

#endif
