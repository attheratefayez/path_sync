#ifndef __PATH_SYNC_PATH_FINDER_HPP__
#define __PATH_SYNC_PATH_FINDER_HPP__

#include "path_sync/grid.hpp"
#include "path_sync/solver.hpp"
#include <optional>

namespace psync {

class PathFinder
{
  private:
    std::vector<Coordinate> __construct_path(std::map<Coordinate, Coordinate> &node_map, const Coordinate &start, const Coordinate& end);

  public:
    PathFinder()
    {
    }

    std::optional<std::vector<Coordinate>> find_path(ISolver &solver, psync::Grid &grid);
};

} // END OF NAMESPACE PSYNC
#endif // !__PATH_SYNC_PATH_FINDER_HPP__
