#ifndef __PATH_SYNC_SOLVER_HPP__
#define __PATH_SYNC_SOLVER_HPP__

#include <memory>
#include <string_view>

#include "path_sync/performance/performance_mat.hpp"
#include "path_sync/path_sync_types.hpp"
#include "path_sync/visualization_system/grid.hpp"

enum SolverType {
    SingleAgentSolver, 
    MultiAgentSolver
};

class ISolver
{
  public:


    virtual std::string_view get_solver_name() const = 0;
    virtual SolverType get_solver_type() const = 0;

    virtual std::map<Coordinate, Coordinate> solve(path_sync::Grid &grid, Coordinate start, Coordinate goal,
                                                   path_sync::PerformanceMetrics &performance_met) = 0;

    virtual std::optional<std::vector<std::vector<Coordinate>>> solve(path_sync::Grid &grid, std::vector<Coordinate> starts,
                                                   std::vector<Coordinate> goals,
                                                   path_sync::PerformanceMetrics &performance_met) = 0;
};

#endif // __PATH_SYNC_SOLVER_HPP__
