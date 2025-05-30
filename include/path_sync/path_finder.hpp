#ifndef __PATH_SYNC_PATH_FINDER_HPP__
#define __PATH_SYNC_PATH_FINDER_HPP__

#include "path_sync/visualization_system/grid.hpp"
#include "path_sync/performance/performance_mat.hpp"
#include "path_sync/solver_interface.hpp"
#include <optional>

namespace path_sync {

class PathFinder
{
  private:
    PerformanceMetrics __performance_met;
    std::vector<Coordinate> __construct_path(std::map<Coordinate, Coordinate> &node_map, const Coordinate &start, const Coordinate& end);

  public:
    PathFinder()
    {
    }

    std::optional<std::vector<Coordinate>> find_path(ISolver &solver, path_sync::Grid &grid);
    std::stringstream get_performance_data() const { return __performance_met.report(); }
};

} // END OF NAMESPACE path_sync
#endif // !__PATH_SYNC_PATH_FINDER_HPP__
