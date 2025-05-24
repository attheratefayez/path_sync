#ifndef __PATH_SYNC_SOLVER_HPP__
#define __PATH_SYNC_SOLVER_HPP__

#include "path_sync/visualization_system/grid.hpp"
#include "path_sync/performance/performance_mat.hpp"

class ISolver
{
  public:
    virtual std::string get_solver_name() const = 0;
    virtual std::map<Coordinate, Coordinate> solve(psync::Grid &grid, Coordinate start, Coordinate end, psync::PerformanceMetrics& performance_met) = 0;
};

#endif // __PATH_SYNC_SOLVER_HPP__
