#ifndef __PATH_SYNC_SOLVER_HPP__
#define __PATH_SYNC_SOLVER_HPP__

#include <map>
#include <string_view>

#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/performance_mat.hpp"

/**
 * @class ISolver
 * @brief Solver Interface for Single Agent Solver
 *
 */
class ISolver
{
  public:
    virtual std::string_view get_solver_name() const = 0;

    virtual std::map<path_sync::Coordinate, path_sync::Coordinate> solve(
        const path_sync::MapData &map_data, path_sync::Coordinate start, path_sync::Coordinate goal,
        path_sync::PerformanceMetrics &performance_met) = 0;
};

/**
 * @class IMASolver
 * @brief Solver Interface for Multi-Agnet Solver
 *
 */
class IMASolver
{
  public:
    virtual std::string_view get_solver_name() const = 0;

    // TODO: chage signature for solve later
    virtual std::optional<std::vector<std::vector<path_sync::Coordinate>>> solve(
        const path_sync::MapData &map_data, std::vector<path_sync::Coordinate> starts,
        std::vector<path_sync::Coordinate> goals, path_sync::PerformanceMetrics &performance_met) = 0;
};


#endif // __PATH_SYNC_SOLVER_HPP__
