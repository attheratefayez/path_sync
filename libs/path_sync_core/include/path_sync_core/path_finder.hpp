#ifndef __PATH_SYNC_PATH_FINDER_HPP__
#define __PATH_SYNC_PATH_FINDER_HPP__

#include <variant>

#include "path_sync_core/performance_mat.hpp"
#include "path_sync_core/solver_interface.hpp"
#include "path_sync_core/solvers/astar_joint_state.hpp"
#include "path_sync_core/solvers/astar_solver.hpp"
#include "path_sync_core/solvers/bfs_solver.hpp"
#include "path_sync_core/solvers/jps_solver.hpp"
#include "path_sync_core/solvers/theta_star_solver.hpp"
#include "path_sync_core/solvers/hpa_solver.hpp"
#include "path_sync_core/solvers/dstar_lite_solver.hpp"
#include "path_sync_core/solvers/epea_solver.hpp"
#include "path_sync_core/solvers/mstar_solver.hpp"

namespace path_sync
{

class PathFinder
{
public:
    PathFinder();

    void change_solver(bool multi_agent = false);
    void select_solver_by_index(std::size_t index);
    std::vector<std::string> get_all_solver_names() const;
    std::vector<std::string> get_sa_solver_names() const;
    std::vector<std::string> get_ma_solver_names() const;
    void select_sa_solver_by_index(std::size_t index);
    void select_ma_solver_by_index(std::size_t index);

    // TODO: implement logic to run the solve for a selected algorithm, or to test env
    // for all algorithms. Like,
    // find_path(map_data, starts, ends, run_on = "astar_solver")
    // find_path(map_data, starts, ends, run_on = "bfs_solver")
    // find_path(map_data, starts, ends, run_on = "test_all")

    [[nodiscard]] std::variant<std::vector<Coordinate>, std::vector<std::vector<Coordinate>>> find_path(
        const path_sync::MapData &map_data, const std::vector<Coordinate> &start_points,
        const std::vector<Coordinate> &end_points);

    std::string_view get_current_solver_name() const
    {
        if (current_sa_solver_)
            return current_sa_solver_->get_solver_name();
        if (current_ma_solver_)
            return current_ma_solver_->get_solver_name();
        return "No Solver Selected";
    }

    std::stringstream get_performance_data() const
    {
        return performance_met_.report();
    }

    PerformanceMetrics get_performance_metrics() const
    {
        return performance_met_;
    }

    void set_scene_id(int id)
    {
        performance_met_.scene_id = id;
    }

private:
    PerformanceMetrics performance_met_;

    ISolver *current_sa_solver_;
    IMASolver *current_ma_solver_;
    std::size_t current_solver_index_;

    std::vector<ISolver *> sa_solvers_;
    std::vector<IMASolver *> ma_solvers_;

    path_sync::solvers::sapf::Astar_Solver astar_solver_;
    path_sync::solvers::sapf::BFS_Solver bfs_solver_;
    path_sync::solvers::sapf::JPS_Solver jps_solver_;
    path_sync::solvers::sapf::Theta_Star_Solver theta_star_solver_;
    path_sync::solvers::sapf::HPA_Solver hpa_solver_;
    path_sync::solvers::sapf::DStar_Lite_Solver dstar_lite_solver_;
    path_sync::solvers::sapf::EPEA_Star_Solver epea_star_solver_;
    path_sync::solvers::mapf::Astar_Joint_State_Solver astar_joint_state_solver;
    path_sync::solvers::mapf::MStar_Solver mstar_solver_;

    std::vector<Coordinate> __construct_path(std::map<Coordinate, Coordinate> &node_map, const Coordinate &start,
                                             const Coordinate &end);
};

} // END OF NAMESPACE path_sync
#endif // !__PATH_SYNC_PATH_FINDER_HPP__
