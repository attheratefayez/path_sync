#ifndef PATH_SYNC_COST_MAP_HPP
#define PATH_SYNC_COST_MAP_HPP

#include <string>
#include <vector>

namespace path_sync
{

struct CostMap
{
    int height = 0;
    int width = 0;
    int objectives = 0;

    std::vector<float> costs;

    float at(int obj, int x, int y) const
    {
        return costs[obj * (height * width) + y * width + x];
    }

    bool is_blocked(int x, int y) const
    {
        for (int o = 0; o < objectives; o++)
            if (at(o, x, y) < 0.0f)
                return true;
        return false;
    }

    bool load(const std::string &filename);
};

} // namespace path_sync

#endif // PATH_SYNC_COST_MAP_HPP
