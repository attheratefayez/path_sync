#ifndef __PATHFINDING_ASTARSOLVER_HPP__
#define __PATHFINDING_ASTARSOLVER_HPP__

#include <stdexcept>

#include "path_sync/performance/performance_mat.hpp"
#include "path_sync/path_sync_types.hpp"
#include "path_sync/solver_interface.hpp"

namespace path_sync
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
    const std::string solver_name_ = "Astar_Solver";
    const SolverType solver_type_ = SolverType::SingleAgentSolver;

  public:
    std::string_view get_solver_name() const override;
    SolverType get_solver_type() const override;
    std::map<Coordinate, Coordinate> solve(path_sync::Grid &grid, Coordinate start, Coordinate goal,
                                           PerformanceMetrics &performance_met) override;

    std::optional<std::vector<std::vector<Coordinate>>> solve(path_sync::Grid &grid, std::vector<Coordinate> starts,
                                           std::vector<Coordinate> goals,
                                           path_sync::PerformanceMetrics &performance_met) override
    {
        throw std::logic_error("Use Solvers from path_sync::solvers::mapf namespace for multi-agent pathfinding.");
    }
};

} // namespace sapf
} // namespace solvers
} // namespace path_sync
#endif
