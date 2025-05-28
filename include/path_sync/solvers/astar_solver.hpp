#ifndef __PATHFINDING_ASTARSOLVER_HPP__
#define __PATHFINDING_ASTARSOLVER_HPP__

#include <memory>
#include <stdexcept>

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
    std::string_view get_solver_name() const override;
    std::map<Coordinate, Coordinate> solve(psync::Grid &grid, Coordinate start, Coordinate goal,
                                           PerformanceMetrics &performance_met) override;

    std::shared_ptr<mapf_type::Node> solve(psync::Grid &grid, std::vector<Coordinate> starts,
                                           std::vector<Coordinate> goals,
                                           psync::PerformanceMetrics &performance_met) override
    {
        throw std::logic_error("Use Solvers from psync::solvers::mapf namespace for multi-agent pathfinding.");
    }
};

} // namespace sapf
} // namespace solvers
} // namespace psync
#endif
