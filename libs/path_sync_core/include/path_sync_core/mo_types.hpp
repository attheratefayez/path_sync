#ifndef PATH_SYNC_MO_TYPES_HPP
#define PATH_SYNC_MO_TYPES_HPP

#include <chrono>
#include <ctime>
#include <sstream>
#include <string>
#include <vector>

#include "path_sync_core/path_sync_types.hpp"

namespace path_sync
{

struct MOSolution
{
    std::vector<Coordinate> path;
    std::vector<float> costs;
    float crowding_distance = 0.0f;

    bool dominates(const MOSolution &other, int num_obj) const
    {
        bool better = false;
        for (int i = 0; i < num_obj; i++)
        {
            if (costs[i] > other.costs[i] + 1e-8f)
                return false;
            if (costs[i] < other.costs[i] - 1e-8f)
                better = true;
        }
        return better;
    }
};

struct MOMetrics
{
    int front_size = 0;
    double hypervolume = 0.0;
    std::vector<float> ref_point;
    std::vector<MOSolution> front;

    std::string report() const
    {
        std::string s;
        s += "Pareto front: " + std::to_string(front_size) + " solutions\n";
        s += "Hypervolume: " + std::to_string(hypervolume) + "\n";
        for (int i = 0; i < front_size && i < 10; i++)
        {
            s += "  #" + std::to_string(i) + " costs: [";
            for (auto c : front[i].costs)
                s += std::to_string(c) + " ";
            s += "]\n";
        }
        return s;
    }

    static std::string csv_header()
    {
        return "mo_solver,map_name,scene_id,front_size,hypervolume,"
               "runtime_us,timestamp";
    }

    std::string csv_line(const std::string &solver, const std::string &map,
                         int scene, bool success,
                         std::chrono::microseconds runtime,
                         std::time_t ts) const
    {
        std::stringstream ss;
        ss << solver << ","
           << map << ","
           << scene << ","
           << front_size << ","
           << hypervolume << ","
           << runtime.count() << ","
           << ts;
        return ss.str();
    }
};

} // namespace path_sync

#endif // PATH_SYNC_MO_TYPES_HPP
