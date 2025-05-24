#ifndef __PATH_SYNC_PSYNC_TYPES__
#define __PATH_SYNC_PSYNC_TYPES__

#include <cstddef>
#include <vector>
#include <SFML/System/Vector2.hpp>

#include "path_sync/visualization_system/cell.hpp"


typedef std::vector<std::vector<psync::Cell>> CellGrid;
typedef std::vector<std::vector<std::vector<size_t>>> CostGrid;
typedef std::pair<int, int> Coordinate;

#endif // !__PATH_SYNC_PSYNC_TYPES__
