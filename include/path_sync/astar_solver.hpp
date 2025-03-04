#ifndef PATHFINDING_ASTARSOLVER_HPP
#define PATHFINDING_ASTARSOLVER_HPP

#include "path_sync/performance_mat.hpp"
#include "path_sync/psync_types.hpp"
#include "path_sync/solver.hpp"

namespace psync
{

class Astar_Solver : public ISolver
{
  private:
    const std::string solver_name = "Astar_Solver";

  public:
    std::string get_solver_name() const override;
    std::map<Coordinate, Coordinate> solve(psync::Grid &grid, Coordinate start, Coordinate end,
                                           PerformanceMetrics &performance_met) override;
};

} // namespace psync
#endif
