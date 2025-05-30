#ifndef __PATHFINDING_BSFSOLVER_HPP__
#define __PATHFINDING_BSFSOLVER_HPP__

#include "path_sync/performance/performance_mat.hpp"
#include "path_sync/path_sync_types.hpp"
#include "path_sync/solver.hpp"
#include "path_sync/visualization_system/grid.hpp"

namespace path_sync
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
    std::string_view get_solver_name() const override;
    std::map<Coordinate, Coordinate> solve(Grid &grid, Coordinate start, Coordinate end,
                                           PerformanceMetrics &performance_met) override;

    mapf_type::NodePtr solve(path_sync::Grid &grid, std::vector<Coordinate> starts, std::vector<Coordinate> goals,
                             path_sync::PerformanceMetrics &performance_met) override
    {
        throw std::logic_error("Use Solvers from path_sync::solvers::mapf namespace for multi-agent pathfinding.");
    }
};

} // namespace sapf
} // namespace solvers
} // namespace path_sync

#endif
