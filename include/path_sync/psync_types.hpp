#ifndef __PATH_SYNC_PSYNC_TYPES__
#define __PATH_SYNC_PSYNC_TYPES__

#include <vector>
#include <SFML/System/Vector2.hpp>
#include "path_sync/cell.hpp"


typedef std::vector<std::vector<psync::Cell>> CellGrid;
typedef std::pair<int, int> Coordinate;

#endif // !__PATH_SYNC_PSYNC_TYPES__
