#include "path_sync_core/path_finder.hpp"
#include "path_sync_core/logger.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/performance_mat.hpp"
#include "path_sync_core/solver_interface.hpp"
#include "path_sync_core/solvers/astar_solver.hpp"
#include "path_sync_core/solvers/bfs_solver.hpp"
#include "path_sync_core/solvers/jps_solver.hpp"
#include "path_sync_core/solvers/theta_star_solver.hpp"
#include "path_sync_core/solvers/hpa_solver.hpp"
#include "path_sync_core/solvers/dstar_lite_solver.hpp"
#include "path_sync_core/solvers/epea_solver.hpp"
#include "path_sync_core/solvers/astar_joint_state.hpp"
#include "path_sync_core/solvers/mstar_solver.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
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

    plugin_loader_.register_sa_solver("Astar_Solver", true,
                                      []() -> std::unique_ptr<ISolver> { return std::make_unique<solvers::sapf::Astar_Solver>(); });
    plugin_loader_.register_sa_solver("BFS_Solver", true,
                                      []() -> std::unique_ptr<ISolver> { return std::make_unique<solvers::sapf::BFS_Solver>(); });
    plugin_loader_.register_sa_solver("JPS_Solver", true,
                                      []() -> std::unique_ptr<ISolver> { return std::make_unique<solvers::sapf::JPS_Solver>(); });
    plugin_loader_.register_sa_solver("Theta_Star_Solver", false,
                                      []() -> std::unique_ptr<ISolver> { return std::make_unique<solvers::sapf::Theta_Star_Solver>(); });
    plugin_loader_.register_sa_solver("HPA_Solver", false,
                                      []() -> std::unique_ptr<ISolver> { return std::make_unique<solvers::sapf::HPA_Solver>(); });
    plugin_loader_.register_sa_solver("DStar_Lite_Solver", true,
                                      []() -> std::unique_ptr<ISolver> { return std::make_unique<solvers::sapf::DStar_Lite_Solver>(); });
    plugin_loader_.register_sa_solver("EPEA_Star_Solver", true,
                                      []() -> std::unique_ptr<ISolver> { return std::make_unique<solvers::sapf::EPEA_Star_Solver>(); });
    plugin_loader_.register_ma_solver("Astar_Joint_State_Solver", true,
                                      []() -> std::unique_ptr<IMASolver> { return std::make_unique<solvers::mapf::Astar_Joint_State_Solver>(); });
    plugin_loader_.register_ma_solver("MStar_Solver", true,
                                      []() -> std::unique_ptr<IMASolver> { return std::make_unique<solvers::mapf::MStar_Solver>(); });

    plugin_loader_.load_plugins("plugins");

    sa_solvers_ = plugin_loader_.get_sa_solvers();
    ma_solvers_ = plugin_loader_.get_ma_solvers();

    if (!sa_solvers_.empty())
        current_sa_solver_ = sa_solvers_[0];
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
    last_ma_metrics_.reset();

    if (timeout_ms_ > 0)
    {
        performance_met_.deadline = std::chrono::steady_clock::now()
                                  + std::chrono::milliseconds(timeout_ms_);
        performance_met_.has_timeout = true;
    }
    else
    {
        performance_met_.has_timeout = false;
    }

    if (start_points.size() != end_points.size())
    {
        throw std::logic_error("Start and End points are not equal.");
    }

    if (start_points.size() == 1 and end_points.size() == 1)
    {
        std::map<Coordinate, Coordinate> node_map;
        std::vector<Coordinate> path;

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

    MAPFMetrics mapf_met;
    performance_met_.mapf_metrics = &mapf_met;

    auto paths = current_ma_solver_->solve(map_data, start_points, end_points, performance_met_);

    performance_met_.mapf_metrics = nullptr;

    if (!paths.has_value())
    {
        ss << "No possible Solution...";
        path_sync::Logger::get().info(ss.str().c_str());
        ss.str(std::string());
        return std::vector<Coordinate>();
    }

    const auto &result = paths.value();

    std::size_t soc = 0;
    std::size_t makespan = 0;
    std::vector<std::size_t> path_lengths;
    path_lengths.reserve(result.size());
    for (const auto &agent_path : result)
    {
        std::size_t len = agent_path.size() - 1;
        soc += len;
        path_lengths.push_back(len);
        if (len > makespan)
            makespan = len;
    }
    performance_met_.sum_of_costs = soc;
    performance_met_.makespan = makespan;

    mapf_met.individual_path_lengths = std::move(path_lengths);
    mapf_met.compute_aggregates();
    last_ma_metrics_ = mapf_met;

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
