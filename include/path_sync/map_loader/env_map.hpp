#ifndef __PSYNC_ENV_MAP_HPP__
#define __PSYNC_ENV_MAP_HPP__

#include <sstream>
#include <string>

#include "path_sync/logging/logger.hpp"
#include "path_sync/map_loader/map_scene.hpp"

namespace psync
{

/**
 * @class Map
 * @brief Contains a map with its scenes
 *
 * @details 
 * A map contains free space and obstacles. 
 * A scene in the map is a a set of numbers defining teh starting and ending positions.
 *
 */
class Map
{
  public:
    Map() = default;
    /**
     * @brief creates a map with map_name.
     *
     * @param map_name 
     */
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

    /**
     * @brief Returns the plain map as a string stream
     *
     * @return 
     */
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

    /**
     * @brief A bucket is a collection of map scenes. 
     * see doc of this class to learn about scenes.
     *
     * @return 
     */
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
