#ifndef __PATH_SYNC_PERFORMANCE_MET_HPP__
#define __PATH_SYNC_PERFORMANCE_MET_HPP__

#include <chrono>
#include <cstdint>
#include <sstream>
#include <string>

namespace path_sync
{

struct PerformanceMetrics
{
    std::string solver_name;
    std::string map_name;
    std::chrono::milliseconds runtime;
    uint16_t path_length;
    uint16_t num_of_nodes_explored;
    uint16_t num_of_nodes_expanded;

    std::stringstream report() const
    {
        std::stringstream ss;
        ss << solver_name << " " << map_name << " rt: " << runtime.count() << " len: " << path_length << " nodes_explored: " << num_of_nodes_explored
           << " nodes_expanded: " << num_of_nodes_expanded << std::endl;

        return ss;
    }
};

} // namespace path_sync
#endif // !__PATH_SYNC_PERFORMANCE_MET_HPP__
