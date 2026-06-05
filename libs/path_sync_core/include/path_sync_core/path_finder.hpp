#ifndef __PATH_SYNC_PATH_FINDER_HPP__
#define __PATH_SYNC_PATH_FINDER_HPP__

#include <atomic>
#include <variant>

#include "path_sync_core/performance_mat.hpp"
#include "path_sync_core/plugin_loader.hpp"
#include "path_sync_core/solver_interface.hpp"

namespace path_sync
{

class PathFinder
{
public:
    PathFinder();

    void change_solver(bool multi_agent = false);
    void select_solver_by_index(std::size_t index);
    std::vector<std::string> get_all_solver_names() const;
    std::vector<std::string> get_sa_solver_names() const;
    std::vector<std::string> get_ma_solver_names() const;
    void select_sa_solver_by_index(std::size_t index);
    void select_ma_solver_by_index(std::size_t index);

    void cancel() { cancel_flag_.store(true); }
    void reset_cancel() { cancel_flag_.store(false); }
    bool is_cancelled() const { return cancel_flag_.load(); }

    [[nodiscard]] std::variant<std::vector<Coordinate>, std::vector<std::vector<Coordinate>>> find_path(
        const path_sync::MapData &map_data, const std::vector<Coordinate> &start_points,
        const std::vector<Coordinate> &end_points);

    std::string_view get_current_solver_name() const
    {
        if (current_sa_solver_)
            return current_sa_solver_->get_solver_name();
        if (current_ma_solver_)
            return current_ma_solver_->get_solver_name();
        return "No Solver Selected";
    }

    std::stringstream get_performance_data() const
    {
        return performance_met_.report();
    }

    PerformanceMetrics get_performance_metrics() const
    {
        return performance_met_;
    }

    void set_scene_id(int id)
    {
        performance_met_.scene_id = id;
    }

    bool is_sa_solver_optimal(std::size_t index) const
    {
        if (index >= sa_solvers_.size()) return false;
        return sa_solvers_[index]->is_optimal();
    }

    bool is_ma_solver_optimal(std::size_t index) const
    {
        if (index >= ma_solvers_.size()) return false;
        return ma_solvers_[index]->is_optimal();
    }

private:
    std::atomic<bool> cancel_flag_{false};
    PerformanceMetrics performance_met_;

    ISolver *current_sa_solver_;
    IMASolver *current_ma_solver_;
    std::size_t current_solver_index_;

    std::vector<ISolver *> sa_solvers_;
    std::vector<IMASolver *> ma_solvers_;

    PluginLoader plugin_loader_;

    std::vector<Coordinate> __construct_path(std::map<Coordinate, Coordinate> &node_map, const Coordinate &start,
                                             const Coordinate &end);
};

} // END OF NAMESPACE path_sync
#endif // !__PATH_SYNC_PATH_FINDER_HPP__
