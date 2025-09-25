#ifndef __PATH_SYNC_MAP_SCENE_HPP__
#define __PATH_SYNC_MAP_SCENE_HPP__

#include <filesystem>
#include <map>
#include <string>
#include <variant>

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
     * @brief Get the Agent_Pool in Bucket n.
     *
     * @param n : int
     *
     * @return :
     * if nth index in inbound, returns the nth agent_pool.
     * otherwise returns the bucket_agent_mapping_'s size.
     */
    std::variant<std::pair<std::vector<Coordinate>, std::vector<Coordinate>>, int> get_nth_bucket(int n)
    {
        std::size_t map_size = bucket_agent_mapping_.size();

        if (n >= 0 and n < map_size)
            return bucket_agent_mapping_[n];

        return static_cast<int>(map_size);
    }

  private:
    std::string map_name_;
    std::vector<std::pair<Coordinate, Coordinate>> agent_pool_;
    std::map<int, std::pair<std::vector<Coordinate>, std::vector<Coordinate>>>
        bucket_agent_mapping_; // bucket_no: <start_positions, end_positions>
    //
    void read_scene_(std::filesystem::path scene_file_path);
};

} // namespace path_sync
#endif // !__PATH_SYNC_MAP_SCENE_HPP__
