#ifndef __PATH_SYNC_APP_HPP__
#define __PATH_SYNC_APP_HPP__

#include <memory>
#include <string>
#include <vector>

#include "path_sync_core/logger.hpp"
#include "path_sync_core/map_loader/env_map.hpp"
#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/path_finder.hpp"
#include "path_sync_core/path_sync_types.hpp"

namespace path_sync {

class PathSyncApp {
public:
    PathSyncApp(int width, int height);

    void run();
    void solve();
    void run_headless_experiments();

    void load_map(const std::string& map_name);
    void load_scenario(int num_agents);

    void add_start_point(Coordinate point);
    void add_end_point(Coordinate point);
    void clear_paths();
    void reset_grid();
    void change_map();
    void change_solver();

    const path_sync::MapData& get_map_data() const { return map_data_; }
    const std::vector<Coordinate>& get_start_points() const { return start_points_; }
    const std::vector<Coordinate>& get_end_points() const { return end_points_; }
    int get_current_map_width() const { return current_map_width_; }
    int get_current_map_height() const { return current_map_height_; }
    std::stringstream get_performance_data() const { return path_finder_.get_performance_data(); }

private:
    path_sync::Map map_loader_;
    path_sync::MapData map_data_;
    path_sync::PathFinder path_finder_;

    std::vector<Coordinate> start_points_;
    std::vector<Coordinate> end_points_;
    int current_map_width_;
    int current_map_height_;
};

} // namespace path_sync

#endif // __PATH_SYNC_APP_HPP__
