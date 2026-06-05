#ifndef __PATHFINDING_MSTAR_SOLVER_HPP__
#define __PATHFINDING_MSTAR_SOLVER_HPP__

#include <optional>
#include <string_view>
#include <vector>

#include "path_sync_core/performance_mat.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/solver_interface.hpp"

namespace path_sync
{
namespace solvers
{
namespace mapf
{

class MStar_Solver : public IMASolver
{
  private:
    const std::string solver_name_ = "MStar_Solver";

    struct JointState
    {
        std::vector<Coordinate> positions;
        int timestep;
    };

    static std::vector<std::vector<Coordinate>> individual_paths(
        const MapData &map_data,
        const std::vector<Coordinate> &starts,
        const std::vector<Coordinate> &goals);

    static std::optional<int> find_first_conflict(
        const std::vector<std::vector<Coordinate>> &paths);

    static std::vector<int> get_conflicted_agents(
        const std::vector<std::vector<Coordinate>> &paths,
        int conflict_time);

    static std::optional<std::vector<std::vector<Coordinate>>> joint_search(
        const MapData &map_data,
        const std::vector<int> &agent_indices,
        const std::vector<Coordinate> &starts,
        const std::vector<Coordinate> &goals,
        const std::vector<std::vector<Coordinate>> &fixed_paths,
        int from_timestep,
        PerformanceMetrics &performance_met);

  public:
    std::string_view get_solver_name() const override;
    bool is_optimal() const override { return false; }

    std::optional<std::vector<std::vector<Coordinate>>> solve(
        const MapData &map_data,
        std::vector<Coordinate> starts,
        std::vector<Coordinate> goals,
        PerformanceMetrics &performance_met) override;
};

} // namespace mapf
} // namespace solvers
} // namespace path_sync

#endif
