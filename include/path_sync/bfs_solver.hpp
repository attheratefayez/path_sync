#ifndef PATHFINDING_BSFSOLVER_HPP
#define PATHFINDING_BSFSOLVER_HPP

#include "path_sync/performance_mat.hpp"
#include "path_sync/psync_types.hpp"
#include "path_sync/solver.hpp"
#include "path_sync/grid.hpp"

namespace psync{

class BFS_Solver : public ISolver
{
  private:
    const std::string solver_name = "BFS_Solver";

  public:
    std::string get_solver_name() const override;
    std::map<Coordinate, Coordinate> solve(Grid &grid, Coordinate start, Coordinate end, PerformanceMetrics& performance_met) override;
};


}
#endif
