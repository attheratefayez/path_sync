#ifndef __PATH_SYNC_MAP_SCENE_HPP__
#define __PATH_SYNC_MAP_SCENE_HPP__

#include <map>
#include <string>

#include "path_sync/psync_types.hpp"
namespace psync 
{

/**
 * @class Scene
 * @brief A scene is a set of numbers representing start_position, end_position on a specific map.
 *
 */
class Scene
{
  public:
    Scene() = default;
    Scene(std::string);

    std::vector<std::pair<Coordinate, Coordinate>> get_n_agent(int no_of_agenets);

    std::map<int, std::pair<std::vector<Coordinate>, std::vector<Coordinate>>> get_bucket_agent_map()
    {
        return bucket_agent_mapping;
    }

  private:
    std::string map_name;
    std::vector<std::pair<Coordinate, Coordinate>> agent_pool;

    /*TODO: MAKE PAIR A TUPLE TO INCLUDE THE OPTIMAL PATH LENGTH FROM SCENE FILE
     * std::map<int, std::tuple<std::vector<Coordinate>, std::vector<Coordinate>, float>>
*/
    std::map<int, std::pair<std::vector<Coordinate>, std::vector<Coordinate>>>
        bucket_agent_mapping; // bucket_no: <start_positions, end_positions>
};


}
#endif // !__PATH_SYNC_MAP_SCENE_HPP__
