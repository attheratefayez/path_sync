#ifndef __PATH_SYNC_ASTAR_JOINT_STATE_SOLVER_HPP__
#define __PATH_SYNC_ASTAR_JOINT_STATE_SOLVER_HPP__

#include <stdexcept>

#include "path_sync/performance/performance_mat.hpp"
#include "path_sync/path_sync_types.hpp"
#include "path_sync/solver_interface.hpp"

namespace path_sync
{
namespace solvers
{
namespace mapf
{

/**
 * @class Astar_Joint_State_Solver
 * @brief Solver Class that uses A*-joint-state-space algorithm to get a solution.
 *
 */
class Astar_Joint_State_Solver : public ISolver
{
  private:
    const std::string solver_name_ = "Astar_Joint_State_Solver";
    const SolverType solver_type_ = SolverType::MultiAgentSolver;

  public:
    std::string_view get_solver_name() const override;
    SolverType get_solver_type() const override;

    std::map<Coordinate, Coordinate> solve(path_sync::Grid &grid, Coordinate start, Coordinate goal,
                                           PerformanceMetrics &performance_met) override
    {
        throw std::logic_error("Use Solvers from path_sync::solvers::sapf namespace for single-agent pathfinding.");
    }

    std::optional<std::vector<std::vector<Coordinate>>> solve(path_sync::Grid &grid, std::vector<Coordinate> starts,
                                           std::vector<Coordinate> goals,
                                           path_sync::PerformanceMetrics &performance_met) override;
};

} // namespace mapf
} // namespace solvers
} // namespace path_sync
#endif
