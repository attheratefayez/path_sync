#ifndef __PATH_SYNC_ENV_MAP_HPP__
#define __PATH_SYNC_ENV_MAP_HPP__

#include <filesystem>
#include <string>

#include "path_sync_core/map_loader/map_scene.hpp"
#include "path_sync_core/path_sync_types.hpp"

namespace path_sync
{

/**
 * @class Map
 * @brief Contains a map with its scenes
 *
 * @details
 * A map contains free space and obstacles.
 * A scene in the map is a a set of numbers defining the starting and ending
 * positions.
 *
 */
class Map
{
  public:
    Map()
        : map_info_{}
        , map_scenes_{}
    {
    }
    /**
     * @brief creates a map with map_name.
     *
     * @param map_name: string -> name of the map to load (without any file extension)
     */
    Map(std::string map_name);

    Map(Map &&other);
    Map &operator=(Map &&other);

    // deleting copy constructor and copy assignment operator
    Map(const Map &) = delete;
    Map &operator=(const Map &) = delete;

    MapInfo const &get_map_info() const
    {
        return map_info_;
    }

    Scene const &get_map_scenes() const
    {
        return map_scenes_;
    }

  private:
    MapInfo map_info_;
    Scene map_scenes_;

    void read_map_(std::filesystem::path map_path);

};

} // namespace path_sync

#endif // !__path_sync_ENV_MAP_HPP__
