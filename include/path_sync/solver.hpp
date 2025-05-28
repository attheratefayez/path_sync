#ifndef __PATH_SYNC_SOLVER_HPP__
#define __PATH_SYNC_SOLVER_HPP__

#include <memory>

#include "path_sync/performance/performance_mat.hpp"
#include "path_sync/psync_types.hpp"
#include "path_sync/visualization_system/grid.hpp"

class ISolver
{
  public:
    virtual std::string get_solver_name() const = 0;
    virtual std::map<Coordinate, Coordinate> solve(psync::Grid &grid, Coordinate start, Coordinate goal,
                                                   psync::PerformanceMetrics &performance_met) = 0;

    virtual std::shared_ptr<mapf_type::Node> solve(psync::Grid &grid, std::vector<Coordinate> starts,
                                                   std::vector<Coordinate> goals,
                                                   psync::PerformanceMetrics &performance_met) = 0;
};

#endif // __PATH_SYNC_SOLVER_HPP__
