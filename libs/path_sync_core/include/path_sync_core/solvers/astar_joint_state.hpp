#ifndef __PATH_SYNC_ASTAR_JOINT_STATE_SOLVER_HPP__
#define __PATH_SYNC_ASTAR_JOINT_STATE_SOLVER_HPP__

#include "path_sync_core/performance_mat.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/solver_interface.hpp"

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
class Astar_Joint_State_Solver : public IMASolver
{
  private:
    const std::string solver_name_ = "Astar_Joint_State_Solver";

  public:
    std::string_view get_solver_name() const override;

    std::optional<std::vector<std::vector<Coordinate>>> solve(const path_sync::MapData &map_data, std::vector<Coordinate> starts,
                                           std::vector<Coordinate> goals,
                                           path_sync::PerformanceMetrics &performance_met) override;
};

} // namespace mapf
} // namespace solvers
} // namespace path_sync
#endif
