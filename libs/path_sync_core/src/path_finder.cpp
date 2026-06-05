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
    : cancel_flag_{false}
    , performance_met_{}
    , current_sa_solver_(nullptr)
    , current_ma_solver_(nullptr)
    , current_solver_index_(0)
{
    performance_met_.cancel_flag = &cancel_flag_;

    sa_solvers_.push_back(&astar_solver_);
    sa_solvers_.push_back(&bfs_solver_);
    sa_solvers_.push_back(&jps_solver_);
    sa_solvers_.push_back(&theta_star_solver_);
    sa_solvers_.push_back(&hpa_solver_);
    sa_solvers_.push_back(&dstar_lite_solver_);
    sa_solvers_.push_back(&epea_star_solver_);
    ma_solvers_.push_back(&astar_joint_state_solver);
    ma_solvers_.push_back(&mstar_solver_);

    // Initialize current_solver_ to the first single-agent solver
    if (!sa_solvers_.empty())
    {
        current_sa_solver_ = sa_solvers_[0];
    }
}

void PathFinder::change_solver(bool multi_agent)
{
    if (multi_agent)
    {
        if (ma_solvers_.empty())
        {
            path_sync::Logger::get().warn("No multi-agent solvers available to change.");
            return;
        }
        auto it = std::find(ma_solvers_.begin(), ma_solvers_.end(), current_ma_solver_);
        std::size_t idx = (it != ma_solvers_.end())
            ? ((it - ma_solvers_.begin()) + 1) % ma_solvers_.size()
            : 0;
        current_ma_solver_ = ma_solvers_[idx];
        current_sa_solver_ = nullptr;
    }
    else
    {
        if (sa_solvers_.empty())
        {
            path_sync::Logger::get().warn("No single-agent solvers available to change.");
            return;
        }
        auto it = std::find(sa_solvers_.begin(), sa_solvers_.end(), current_sa_solver_);
        std::size_t idx = (it != sa_solvers_.end())
            ? ((it - sa_solvers_.begin()) + 1) % sa_solvers_.size()
            : 0;
        current_sa_solver_ = sa_solvers_[idx];
        current_ma_solver_ = nullptr;
    }
}

void PathFinder::select_solver_by_index(std::size_t index)
{
    std::size_t total = sa_solvers_.size() + ma_solvers_.size();
    if (total == 0 || index >= total)
        return;
    current_solver_index_ = index;
    if (current_solver_index_ < sa_solvers_.size())
    {
        current_sa_solver_ = sa_solvers_[current_solver_index_];
        current_ma_solver_ = nullptr;
    }
    else
    {
        current_ma_solver_ = ma_solvers_[current_solver_index_ - sa_solvers_.size()];
        current_sa_solver_ = nullptr;
    }
}

void PathFinder::select_sa_solver_by_index(std::size_t index)
{
    if (index >= sa_solvers_.size())
        return;
    current_sa_solver_ = sa_solvers_[index];
    current_ma_solver_ = nullptr;
}

void PathFinder::select_ma_solver_by_index(std::size_t index)
{
    if (index >= ma_solvers_.size())
        return;
    current_ma_solver_ = ma_solvers_[index];
    current_sa_solver_ = nullptr;
}

std::vector<std::string> PathFinder::get_sa_solver_names() const
{
    std::vector<std::string> names;
    for (auto *s : sa_solvers_)
        names.push_back(std::string(s->get_solver_name()));
    return names;
}

std::vector<std::string> PathFinder::get_ma_solver_names() const
{
    std::vector<std::string> names;
    for (auto *s : ma_solvers_)
        names.push_back(std::string(s->get_solver_name()));
    return names;
}

std::vector<std::string> PathFinder::get_all_solver_names() const
{
    std::vector<std::string> names;
    for (auto *s : sa_solvers_)
        names.push_back(std::string(s->get_solver_name()));
    for (auto *s : ma_solvers_)
        names.push_back(std::string(s->get_solver_name()));
    return names;
}

std::variant<std::vector<Coordinate>, std::vector<std::vector<Coordinate>>> PathFinder::find_path(
    const path_sync::MapData &map_data, const std::vector<Coordinate> &start_points,
    const std::vector<Coordinate> &end_points)
{
    std::stringstream ss;

    performance_met_.map_name = map_data.get_map_info().map_name;
    performance_met_.num_agents = static_cast<int>(start_points.size());
    performance_met_.timestamp = std::time(nullptr);

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

    // Multi-agent case
    if (!current_ma_solver_)
    {
        if (ma_solvers_.empty())
        {
            path_sync::Logger::get().warn("No multi-agent solvers available.");
            return std::vector<Coordinate>();
        }
        current_ma_solver_ = ma_solvers_[0];
    }

    performance_met_.solver_name = current_ma_solver_->get_solver_name();
    auto paths = current_ma_solver_->solve(map_data, start_points, end_points, performance_met_);

    if (!paths.has_value())
    {
        ss << "No possible Solution...";
        path_sync::Logger::get().info(ss.str().c_str());
        ss.str(std::string());
        return std::vector<Coordinate>();
    }

    const auto &result = paths.value();

    // Compute sum_of_costs and makespan for MAPF
    std::size_t soc = 0;
    std::size_t makespan = 0;
    for (const auto &agent_path : result)
    {
        soc += agent_path.size() - 1;
        if (agent_path.size() > makespan)
            makespan = agent_path.size() - 1;
    }
    performance_met_.sum_of_costs = soc;
    performance_met_.makespan = makespan;

    return result;
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
