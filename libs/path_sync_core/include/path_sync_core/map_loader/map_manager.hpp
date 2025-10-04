#ifndef __PATH_SYNC_MAP_MANAGER_HPP__
#define __PATH_SYNC_MAP_MANAGER_HPP__

#include <string>
#include <vector>

#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/map_loader/env_map.hpp"
#include "path_sync_core/map_loader/map_data.hpp"

// NOTE:
// should load all the maps in maps dir
// get_current_map() will give currently selected map's mapinfo
// change_map() will select the next map, will return false
// if no more maps available otherwise true
// get_scene(int n_agent) will return scene with n_agent

namespace path_sync 
{

class MapManager
{

public:
    MapManager();

    MapData get_current_map_data();
    MapData get_next_map_data();
    std::pair<std::vector<Coordinate>, std::vector<Coordinate>> get_current_scene() const;
    std::pair<std::vector<Coordinate>, std::vector<Coordinate>> get_next_scene(int n_agent) ;

    void reset();


private:
    std::vector<std::string> available_maps_;
    Map current_map_;
    MapData current_map_data_;
    std::pair<std::vector<Coordinate>, std::vector<Coordinate>> current_scene_;
    mutable int current_map_idx_, current_scene_idx_, total_scenes_;
    
    void _get_available_maps();

};

} // ! END OF NAMESPACE path_sync
#endif // !__PATH_SYNC_MAP_MANAGER_HPP__
