#ifndef __PATH_SYNC_ENV_MAP_HPP__
#define __PATH_SYNC_ENV_MAP_HPP__

#include <filesystem>
#include <sstream>
#include <string>

#include "path_sync/logging/logger.hpp"
#include "path_sync/map_loader/map_scene.hpp"
#include "path_sync/path_sync_types.hpp"

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
    Map() : selected_map_index_(0)
    {
        get_available_maps_();
    } // default constructor
    /**
     * @brief creates a map with map_name.
     *
     * @param map_name
     */
    Map(std::string map_name);

    Map(Map &&other);
    Map &operator=(Map &&other);

    Map(const Map &) = delete;
    Map &operator=(const Map &) = delete;

    inline const MapInfo &get_map_info() const
    {
        return current_map_info_;
    }
    /**
     * @brief A bucket is a collection of map scenes.
     * see doc of this class to learn about scenes.
     *
     * @return
     */
   std::pair<std::vector<Coordinate>, std::vector<Coordinate>> load_n_agents(int n = 1);

  private:
    std::vector<std::string> available_maps_;
    std::size_t selected_map_index_;

    MapInfo current_map_info_;
    Scene map_scenes_;
    int current_bucket_idx_;
    std::pair<std::vector<Coordinate>, std::vector<Coordinate>> current_bucket_;
    int available_agents_in_current_bucket_;

    void get_available_maps_();
    void read_map_(std::filesystem::path map_path);

    Scene &get_map_scenes()
    {
        return map_scenes_;
    }

    std::pair<std::vector<Coordinate>, std::vector<Coordinate>> get_current_bucket_();
    std::pair<std::vector<Coordinate>, std::vector<Coordinate>> next_bucket_();
    std::pair<std::vector<Coordinate>, std::vector<Coordinate>> previous_bucket_();

};

} // namespace path_sync

#endif // !__path_sync_ENV_MAP_HPP__
