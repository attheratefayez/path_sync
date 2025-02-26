#include "path_sync/grid.hpp"

#ifndef __PATH_SYNC_SOLVER_HPP__
#define __PATH_SYNC_SOLVER_HPP__

class ISolver
{
  public:
    virtual std::string get_solver_name() const = 0;
    virtual std::map<Coordinate, Coordinate> solve(psync::Grid &grid, Coordinate start, Coordinate end) = 0;
};

#endif // __PATH_SYNC_SOLVER_HPP__
