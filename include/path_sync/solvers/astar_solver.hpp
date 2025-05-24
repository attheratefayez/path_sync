#ifndef __PATHFINDING_ASTARSOLVER_HPP__
#define __PATHFINDING_ASTARSOLVER_HPP__

#include "path_sync/performance/performance_mat.hpp"
#include "path_sync/psync_types.hpp"
#include "path_sync/solver.hpp"

namespace psync
{
namespace solvers
{
namespace sapf
{

/**
 * @class Astar_Solver
 * @brief Solver Class that uses A* algorithm to get a solution.
 *
 */
class Astar_Solver : public ISolver
{
  private:
    const std::string solver_name = "Astar_Solver";

  public:
    std::string get_solver_name() const override;
    std::map<Coordinate, Coordinate> solve(psync::Grid &grid, Coordinate start, Coordinate end,
                                           PerformanceMetrics &performance_met) override;
};

} // namespace sapf
} // namespace solvers
} // namespace psync
#endif
