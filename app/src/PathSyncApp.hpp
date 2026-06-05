#ifndef __PATH_SYNC_APP_HPP__
#define __PATH_SYNC_APP_HPP__

#include <mutex>
#include <vector>
#include <memory>

#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/map_loader/map_manager.hpp"
#include "path_sync_core/path_finder.hpp"
#include "path_sync_core/path_sync_types.hpp"
#include "path_sync_core/performance_mat.hpp"

namespace path_sync
{

class PathSyncApp
{
  public:
    PathSyncApp();

    bool request_next_map();
    bool request_previous_map();
    bool request_map(int map_idx);
    bool request_next_scene();
    bool request_previous_scene();
    bool request_scene(int scene_index);
    bool solve_current_scene();
    bool solve_current_map();

    // Thread-safe: works on a copy, locks internal mutex.
    // Returns the solved map with PATH cells applied, or nullptr on failure.
    std::shared_ptr<path_sync::MapData> solve_async_on_copy(
        const std::vector<Coordinate>& starts,
        const std::vector<Coordinate>& ends);

    std::pair<std::vector<Coordinate>, std::vector<Coordinate>> get_current_scene() const;

    void set_map_data(std::shared_ptr<path_sync::MapData> data);
    void change_solver();
    void select_solver_by_index(std::size_t index);
    void select_solver_by_index(std::size_t index, bool multi_agent);
    std::vector<std::string> get_solver_names() const;
    std::vector<std::string> get_solver_names(bool multi_agent) const;
    bool get_is_multi_agent() const;
    void toggle_agent_mode();

    std::shared_ptr<path_sync::MapData> get_current_map_data() const;

    void cancel_solve() { path_finder_.cancel(); }
    void reset_cancel() { path_finder_.reset_cancel(); }
    bool is_solve_cancelled() const { return path_finder_.is_cancelled(); }

    void clear_scene();
    void clear_paths();
    void reset_grid();

    std::string_view get_current_solver_name() const;
    bool is_solver_optimal(std::size_t index, bool multi_agent) const;
    int get_scene_index() const;
    int get_total_scenes() const;
    int get_map_index() const;
    int get_total_maps() const;
    std::string get_current_map_name() const;
    int get_num_agents() const;
    PerformanceMetrics get_performance_metrics() const;
   
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
    mutable std::mutex solve_mutex_;

    int num_agents_;

    void update_map_data_with_current_scene_();

};

} // namespace path_sync

#endif // __PATH_SYNC_APP_HPP__
