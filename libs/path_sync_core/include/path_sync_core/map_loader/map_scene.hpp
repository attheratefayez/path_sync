#ifndef __PATH_SYNC_MAP_SCENE_HPP__
#define __PATH_SYNC_MAP_SCENE_HPP__

#include <filesystem>
#include <string>

#include "path_sync_core/path_sync_types.hpp"

namespace path_sync
{

/**
 * @class Scene
 * @brief A scene is a set of numbers representing start_position, end_position on a specific map.
 * Each Scene has several Buckets.
 * In Each Bucket, there are starting and ending positions for no more than 10 agents.
 * Starting and ending positions in a bucket is the Agent_Pool. Client can take out any
 * number of agents they might require.
 *
 */
class Scene
{
  public:
    Scene() = default;
    Scene(std::string map_name);

    Scene(Scene &&map_scene);
    Scene &operator=(Scene &&map_scene);

    Scene(const Scene &) = delete;
    Scene &operator=(const Scene &) = delete;
    

    /**
     * @brief getter for scene_data_
     *
     * @return scene_data_ : std::vector<std::pair<Coordinate, Coordinate>> 
     */
    std::vector<std::pair<Coordinate, Coordinate>> const& get_scene_data() const 
    {
        return scene_data_;
    }

  private:
    std::string map_name_;
    std::vector< std::pair<Coordinate, Coordinate>> scene_data_; // pair <start_positions, end_positions>

    void read_scene_(std::filesystem::path scene_file_path);
};

} // namespace path_sync
#endif // !__PATH_SYNC_MAP_SCENE_HPP__
