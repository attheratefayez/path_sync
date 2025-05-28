#ifndef __PATHFINDING_ASTAR_JOINT_STATE_SOLVER_HPP__
#define __PATHFINDING_ASTAR_JOINT_STATE_SOLVER_HPP__

#include <memory>
#include <stdexcept>

#include "path_sync/performance/performance_mat.hpp"
#include "path_sync/psync_types.hpp"
#include "path_sync/solver.hpp"

namespace psync
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
    const std::string solver_name = "Astar_Joint_State_Solver";

  public:
    std::string get_solver_name() const override;

    std::map<Coordinate, Coordinate> solve(psync::Grid &grid, Coordinate start, Coordinate goal,
                                           PerformanceMetrics &performance_met) override
    {
        throw std::logic_error("Use Solvers from psync::solvers::sapf namespace for single-agent pathfinding.");
    }

    std::shared_ptr<mapf_type::Node> solve(psync::Grid &grid, std::vector<Coordinate> starts,
                                           std::vector<Coordinate> goals,
                                           psync::PerformanceMetrics &performance_met) override;
};

} // namespace mapf
} // namespace solvers
} // namespace psync
#endif
