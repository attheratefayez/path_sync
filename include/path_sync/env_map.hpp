#ifndef __PSYNC_ENV_MAP_HPP__
#define __PSYNC_ENV_MAP_HPP__

#include <sstream>
#include <string>

#include "path_sync/logger.hpp"
#include "path_sync/map_scene.hpp"

namespace psync
{

class Map
{
  public:
    Map() = default;
    Map(std::string map_name);
    Map(Map &);
    Map& operator=(const Map& obj);

    int get_map_height() const
    {
        return __map_height;
    }

    int get_map_width() const
    {
        return __map_width;
    }

    std::stringstream &get_map()
    {
        return __map;
    }

    std::string &get_map_name()
    {
        return __map_name;
    }

    Scene &get_map_scenes()
    {
        return __map_scenes;
    }

    int get_current_bucket() const
    {
        return __current_bucket;
    }

    int next_bucket()
    {
        if (__current_bucket < __map_scenes.get_bucket_agent_map().size())
        {
            __current_bucket++;
        }

        else
        {
            psync::Logger::get()->warn("No next bucket.");
        }

        return __current_bucket;
    }

    int previous_bucket()
    {
        if (__current_bucket > 0)
        {
            __current_bucket--;
        }
        else
        {
            psync::Logger::get()->warn("No previous bucket.");
        }

        return __current_bucket;
    }

  private:
    int __map_height;
    int __map_width;
    std::stringstream __map;
    std::string __map_name;
    int __current_bucket;
    Scene __map_scenes;
};

} // END OF NAMESPACE psync

#endif // !__PSYNC_ENV_MAP_HPP__
