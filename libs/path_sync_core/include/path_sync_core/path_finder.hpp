#ifndef __PATH_SYNC_PATH_FINDER_HPP__
#define __PATH_SYNC_PATH_FINDER_HPP__

#include <unistd.h>
#include <variant>

#include "path_sync_core/performance_mat.hpp"
#include "path_sync_core/solver_interface.hpp"
#include "path_sync_core/solvers/astar_joint_state.hpp"
#include "path_sync_core/solvers/astar_solver.hpp"
#include "path_sync_core/solvers/bfs_solver.hpp"

namespace path_sync
{

class PathFinder
{
  private:
    PerformanceMetrics performance_met_;

    ISolver *current_solver_;
    std::size_t current_solver_index_;

    std::vector<ISolver *> sa_solvers_;
    std::vector<IMASolver *> ma_solvers_;

    path_sync::solvers::sapf::Astar_Solver astar_solver_;
    path_sync::solvers::sapf::BFS_Solver bfs_solver_;
    path_sync::solvers::mapf::Astar_Joint_State_Solver astar_joint_state_solver;

    std::vector<Coordinate> __construct_path(std::map<Coordinate, Coordinate> &node_map, const Coordinate &start,
                                             const Coordinate &end);

  public:
    PathFinder();

    void change_solver();

    // TODO: implement logic to run the solve for a selected algorithm, or to test env 
    // for all algorithms. Like, 
    // find_path(map_data, starts, ends, run_on = "astar_solver")
    // find_path(map_data, starts, ends, run_on = "bfs_solver")
    // find_path(map_data, starts, ends, run_on = "test_all")

    std::variant<std::vector<Coordinate>, std::vector<std::vector<Coordinate>>> find_path(
        const path_sync::MapData &map_data, const std::vector<Coordinate> &start_points,
        const std::vector<Coordinate> &end_points);

    std::stringstream get_performance_data() const
    {
        return performance_met_.report();
    }
};

} // END OF NAMESPACE path_sync
#endif // !__PATH_SYNC_PATH_FINDER_HPP__
