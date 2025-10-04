#include "path_sync_core/path_finder.hpp"
#include "path_sync_core/logger.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/performance_mat.hpp"
#include "path_sync_core/solver_interface.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace path_sync
{
PathFinder::PathFinder()
    : performance_met_{}
    , current_sa_solver_(nullptr)
    , current_ma_solver_(nullptr)
    , current_solver_index_(0)
{
    sa_solvers_.push_back(&astar_solver_);
    sa_solvers_.push_back(&bfs_solver_);
    ma_solvers_.push_back(&astar_joint_state_solver);

    // Initialize current_solver_ to the first single-agent solver
    if (!sa_solvers_.empty())
    {
        current_sa_solver_ = sa_solvers_[0];
    }
}

void PathFinder::change_solver()
{
    if (sa_solvers_.empty())
    {
        path_sync::Logger::get().warn("No single-agent solvers available to change.");
        return;
    }

    current_solver_index_ = (++current_solver_index_) % sa_solvers_.size();
    current_sa_solver_ = sa_solvers_[current_solver_index_];
}

std::variant<std::vector<Coordinate>, std::vector<std::vector<Coordinate>>> PathFinder::find_path(
    const path_sync::MapData &map_data, const std::vector<Coordinate> &start_points,
    const std::vector<Coordinate> &end_points)
{
    std::stringstream ss;

    if (start_points.size() != end_points.size())
    {
        throw std::logic_error("Start and End points are not equal.");
    }

    if (start_points.size() == 1 and end_points.size() == 1)
    {
        std::map<Coordinate, Coordinate> node_map;
        std::vector<Coordinate> path;

        // Ensure current_solver_ is a single-agent solver
        if (!current_sa_solver_)
        {
            path_sync::Logger::get().warn("Solver is not selected yet. Selected the first sa_solver");
            current_sa_solver_ = sa_solvers_[0];
            current_solver_index_ = 0;
        }

        performance_met_.solver_name = current_sa_solver_->get_solver_name();

        node_map = current_sa_solver_->solve(map_data, start_points[0], end_points[0], performance_met_);

        if (node_map.empty())
        {
            ss << "No possible route from: (" << start_points[0].second << ", " << start_points[0].first << ") to : ("
               << end_points[0].second << ", " << end_points[0].first << ")." << std::endl;

            path_sync::Logger::get().info(ss.str().c_str());
            ss.str(std::string());

            return std::vector<Coordinate>();
        }

        path = __construct_path(node_map, start_points[0], end_points[0]);
        performance_met_.path_length = path.size();
        return path;
    }

    // TODO: add logic to handle multi-agent case

    // else
    // {
    //     current_solver_ = solver_map_[SolverType::MultiAgentSolver][0];
    //     std::optional<std::vector<std::vector<Coordinate>>> paths =
    //         current_solver_->solve(map_data, start_points, end_points, performance_met_);
    //
    //     if (!paths.has_value())
    //     {
    //         ss << "No possible Solution...";
    //         path_sync::Logger::get()->info(ss.str().c_str());
    //         ss.str(std::string());
    //         return std::vector<Coordinate>();
    //     }
    //
    //     return paths.value();
    // }

    return std::vector<Coordinate>();
}

std::vector<Coordinate> PathFinder::__construct_path(std::map<Coordinate, Coordinate> &node_map,
                                                     const Coordinate &start, const Coordinate &end)
{
    std::vector<Coordinate> the_path;
    the_path.push_back(end);

    Coordinate current = node_map[end];
    while (current != start)
    {
        the_path.push_back(current);
        current = node_map[current];
    }
    the_path.push_back(start);

    std::reverse(the_path.begin(), the_path.end());
    return std::vector<Coordinate>(the_path.begin(), the_path.end());
}

} // namespace path_sync
