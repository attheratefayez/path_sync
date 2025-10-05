#ifndef __PATH_SYNC_APP_HPP__
#define __PATH_SYNC_APP_HPP__

#include <vector>
#include <memory>

#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/map_loader/map_manager.hpp"
#include "path_sync_core/path_finder.hpp"
#include "path_sync_core/path_sync_types.hpp"

namespace path_sync
{

class PathSyncApp
{
  public:
    PathSyncApp();

    bool request_next_map();
    bool request_next_scene();
    bool solve_current_scene();

    std::shared_ptr<path_sync::MapData const> get_current_map_data() const;

    void clear_scene();
    void clear_paths();
    void reset_grid();
   
    std::stringstream get_performance_data() const
    {
        return path_finder_.get_performance_data();
    }

  private:
    path_sync::MapManager map_manager_;
    std::shared_ptr<path_sync::MapData> current_map_data_;
    std::pair<std::vector<Coordinate>, std::vector<Coordinate>> current_scene_;
    std::vector<Coordinate> current_sa_solution_;
    std::vector<std::vector<Coordinate>> current_ma_solution_;

    path_sync::PathFinder path_finder_;

    int num_agents_;

    void update_map_data_with_current_scene_();

};

} // namespace path_sync

#endif // __PATH_SYNC_APP_HPP__
