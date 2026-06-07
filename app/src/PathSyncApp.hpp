#ifndef __PATH_SYNC_APP_HPP__
#define __PATH_SYNC_APP_HPP__

#include <mutex>
#include <vector>
#include <memory>

#include "path_sync_core/map_loader/cost_map.hpp"
#include "path_sync_core/map_loader/map_data.hpp"
#include "path_sync_core/map_loader/map_manager.hpp"
#include "path_sync_core/mo_types.hpp"
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

    // Standard async solve (SA / MA)
    std::shared_ptr<path_sync::MapData> solve_async_on_copy(
        const std::vector<Coordinate>& starts,
        const std::vector<Coordinate>& ends);

    // Multi-objective async solve
    std::shared_ptr<path_sync::MapData> solve_mo_async_on_copy(
        Coordinate start, Coordinate goal, int num_objectives);

    std::pair<std::vector<Coordinate>, std::vector<Coordinate>> get_current_scene() const;

    void set_map_data(std::shared_ptr<path_sync::MapData> data);
    void change_solver();
    void select_solver_by_index(std::size_t index);
    void select_solver_by_index(std::size_t index, bool multi_agent);
    void select_sa_solver_by_index(std::size_t index) { path_finder_.select_sa_solver_by_index(index); }
    void select_ma_solver_by_index(std::size_t index) { path_finder_.select_ma_solver_by_index(index); }
    std::vector<std::string> get_solver_names() const;
    std::vector<std::string> get_solver_names(bool multi_agent) const;
    bool get_is_multi_agent() const;
    void toggle_agent_mode();
    void set_num_agents(int n);

    // MO solver support
    bool has_mo_solver() const { return path_finder_.has_mo_solver(); }
    bool is_current_solver_mo() const { return path_finder_.is_current_solver_mo(); }
    std::vector<std::string> get_mo_solver_names() const;
    bool is_mo_solver_optimal(std::size_t index) const;
    void select_mo_solver_by_index(std::size_t index);
    std::size_t get_mo_solver_count() const { return path_finder_.get_mo_solver_count(); }
    int get_num_objectives() const { return num_objectives_; }
    void set_num_objectives(int n) { num_objectives_ = n; }

    // MO result access
    const std::vector<MOSolution>& get_current_mo_front() const { return current_mo_front_; }
    const MOMetrics& get_current_mo_metrics() const { return current_mo_metrics_; }
    Coordinate get_current_mo_start() const { return current_mo_start_; }
    Coordinate get_current_mo_goal() const { return current_mo_goal_; }
    int get_current_mo_selection() const { return current_mo_selection_; }
    void select_mo_solution(int index);
    void set_mo_weights(const std::vector<float>& w);
    bool load_cost_map_for_current_map();
    std::shared_ptr<CostMap> load_cost_map();

    std::shared_ptr<path_sync::MapData> get_current_map_data() const;

    void cancel_solve() { path_finder_.cancel(); }
    void reset_cancel() { path_finder_.reset_cancel(); }
    bool is_solve_cancelled() const { return path_finder_.is_cancelled(); }
    void set_timeout_ms(int ms) { path_finder_.set_timeout_ms(ms); }

    bool create_blank_map(int width, int height);
    void save_custom_map();
    void set_start_point(Coordinate c);
    void set_goal_point(Coordinate c);
    void remove_point(Coordinate c);
    void clear_paths();
    void reset_grid();

    std::string_view get_current_solver_name() const;
    bool is_solver_optimal(std::size_t index, bool multi_agent) const;
    bool is_sa_solver_optimal(std::size_t index) const { return path_finder_.is_sa_solver_optimal(index); }
    bool is_ma_solver_optimal(std::size_t index) const { return path_finder_.is_ma_solver_optimal(index); }
    int get_scene_index() const;
    int get_total_scenes() const;
    int get_map_index() const;
    int get_total_maps() const;
    std::string get_current_map_name() const;
    int get_num_agents() const;
    PerformanceMetrics get_performance_metrics() const;
    const std::optional<MAPFMetrics>& get_last_ma_metrics() const
    {
        return path_finder_.get_last_ma_metrics();
    }
    const std::vector<std::vector<Coordinate>> &get_current_ma_solution() const
    {
        return current_ma_solution_;
    }

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

    // MO state
    std::vector<MOSolution> current_mo_front_;
    MOMetrics current_mo_metrics_;
    Coordinate current_mo_start_{0, 0};
    Coordinate current_mo_goal_{0, 0};
    int current_mo_selection_ = 0;
    int num_objectives_ = 5;
    std::vector<float> mo_weights_ = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

    path_sync::PathFinder path_finder_;
    mutable std::mutex solve_mutex_;

    int num_agents_;

    std::shared_ptr<CostMap> generate_cost_map_from_map_data(const MapData &map_data);

    bool is_custom_map_() const;
    void update_map_data_with_current_scene_();

};

} // namespace path_sync
#endif // __PATH_SYNC_APP_HPP__
