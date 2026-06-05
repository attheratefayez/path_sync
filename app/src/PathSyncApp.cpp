#include "PathSyncApp.hpp"
#include "path_sync_core/logger.hpp"
#include "path_sync_core/path_sync_types.hpp"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

namespace path_sync
{

PathSyncApp::PathSyncApp()
    : map_manager_{}
    , path_finder_{}
    , current_scene_{}
    , current_sa_solution_{}
    , current_ma_solution_{}
    , num_agents_{1}
{
    current_map_data_ = std::make_shared<path_sync::MapData>(map_manager_.get_next_map_data());
    current_scene_ = map_manager_.get_next_scene(num_agents_);

    update_map_data_with_current_scene_();

    std::stringstream ss;
    ss << "Current map: " << map_manager_.get_current_map_data().get_map_info().map_name << std::endl;

    path_sync::Logger::get().info(ss.str().c_str());
}

bool PathSyncApp::request_next_map()
{
    auto md = map_manager_.get_next_map_data();
    if (md.get_height() == 0)
        return false;
    current_map_data_ = std::make_shared<path_sync::MapData>(std::move(md));
    current_scene_ = map_manager_.get_next_scene(num_agents_);
    update_map_data_with_current_scene_();
    return true;
}

bool PathSyncApp::request_previous_map()
{
    auto md = map_manager_.get_prev_map_data();
    if (md.get_height() == 0)
        return false;
    current_map_data_ = std::make_shared<path_sync::MapData>(std::move(md));
    current_scene_ = map_manager_.get_next_scene(num_agents_);
    update_map_data_with_current_scene_();
    return true;
}

bool PathSyncApp::request_map(int map_idx)
{
    auto md = map_manager_.set_map_index(map_idx);
    if (md.get_height() == 0)
        return false;
    current_map_data_ = std::make_shared<path_sync::MapData>(std::move(md));
    current_scene_ = map_manager_.get_next_scene(num_agents_);
    update_map_data_with_current_scene_();
    return true;
}

bool PathSyncApp::request_next_scene()
{
    for (auto &elem : current_scene_.first)
        current_map_data_->set_cell_type(elem, path_sync::CellType::DEFAULT);

    for (auto &elem : current_scene_.second)
        current_map_data_->set_cell_type(elem, path_sync::CellType::DEFAULT);

    current_scene_ = map_manager_.get_next_scene(num_agents_);
    update_map_data_with_current_scene_();

    return bool(current_scene_.first.size());
}

bool PathSyncApp::request_scene(int scene_index)
{
    for (auto &elem : current_scene_.first)
        current_map_data_->set_cell_type(elem, path_sync::CellType::DEFAULT);

    for (auto &elem : current_scene_.second)
        current_map_data_->set_cell_type(elem, path_sync::CellType::DEFAULT);

    current_scene_ = map_manager_.set_scene_index(scene_index, num_agents_);
    if (current_scene_.first.empty())
        return false;

    update_map_data_with_current_scene_();
    return true;
}

bool PathSyncApp::request_previous_scene()
{
    for (auto &elem : current_scene_.first)
        current_map_data_->set_cell_type(elem, path_sync::CellType::DEFAULT);

    for (auto &elem : current_scene_.second)
        current_map_data_->set_cell_type(elem, path_sync::CellType::DEFAULT);

    current_scene_ = map_manager_.get_prev_scene(num_agents_);
    update_map_data_with_current_scene_();

    return bool(current_scene_.first.size());
}

std::shared_ptr<path_sync::MapData> PathSyncApp::solve_async_on_copy(
    const std::vector<Coordinate>& starts,
    const std::vector<Coordinate>& ends)
{
    auto copy = std::make_shared<path_sync::MapData>(*current_map_data_);
    int scene_id = std::max(0, map_manager_.get_current_scene_index() - num_agents_);

    std::lock_guard<std::mutex> lock(solve_mutex_);
    path_finder_.set_scene_id(scene_id);
    auto result = path_finder_.find_path(*copy, starts, ends);

    if (std::holds_alternative<std::vector<Coordinate>>(result))
    {
        current_sa_solution_ = std::get<std::vector<Coordinate>>(std::move(result));
        if (current_sa_solution_.empty())
            return nullptr;

        current_sa_solution_ =
            std::vector<Coordinate>(current_sa_solution_.begin() + 1, current_sa_solution_.end() - 1);

        for (Coordinate const &elem : current_sa_solution_)
            copy->set_cell_type(elem, path_sync::CellType::PATH);
    }
    else
    {
        current_ma_solution_ = std::get<std::vector<std::vector<Coordinate>>>(std::move(result));
        if (std::any_of(current_ma_solution_.begin(), current_ma_solution_.end(),
                        [](std::vector<Coordinate> &elem) { return elem.empty(); }))
            return nullptr;

        for (auto &path : current_ma_solution_)
        {
            path = std::vector<Coordinate>(path.begin() + 1, path.end() - 1);

            for (auto &elem : path)
                copy->set_cell_type(elem, path_sync::CellType::PATH);
        }
    }

    return copy;
}

bool PathSyncApp::solve_current_scene()
{
    auto result = solve_async_on_copy(current_scene_.first, current_scene_.second);
    if (!result)
        return false;
    current_map_data_ = std::move(result);
    Logger::get().info(path_finder_.get_performance_data().str().c_str());
    return true;
}

std::pair<std::vector<Coordinate>, std::vector<Coordinate>> PathSyncApp::get_current_scene() const
{
    return current_scene_;
}

void PathSyncApp::set_map_data(std::shared_ptr<path_sync::MapData> data)
{
    current_map_data_ = std::move(data);
}

bool PathSyncApp::solve_current_map()
{
    // map_manager_.reset();

    // NOTE:
    // getting a clear map_data. Because, in initalization, update_map_data_with_current_scene_
    // sets starts and ends in mapdata
    current_map_data_ = std::make_shared<path_sync::MapData>(map_manager_.get_current_map_data());
    current_scene_ = map_manager_.get_next_scene(1);

    std::string log_file_name = "";
    std::string log_file_path = "";

    log_file_name += current_map_data_->get_map_info().map_name;

    auto time_now = std::chrono::system_clock::now();
    auto time_now_c = std::chrono::system_clock::to_time_t(time_now);
    std::tm time_now_tm = *std::localtime(&time_now_c);
    std::stringstream ss;
    ss << std::put_time(&time_now_tm, "%d_%m_%Y-%H:%M");

    log_file_name += ss.str();
    log_file_name += ".txt";
    log_file_path = std::string(PROJECT_ROOT) + "/log/" + log_file_name;
    std::shared_ptr<std::ofstream> log_file = std::make_shared<std::ofstream>(log_file_path);

    while(not current_scene_.first.empty())
    {
        path_finder_.set_scene_id(map_manager_.get_current_scene_index() - 1);
        (void)path_finder_.find_path(*current_map_data_, current_scene_.first, current_scene_.second);
        current_scene_ = map_manager_.get_next_scene(1);

        path_sync::Logger::get().info(path_finder_.get_performance_data().str().c_str(), log_file);
    }

    log_file->close();

    return true;
}

void PathSyncApp::change_solver()
{
    path_finder_.change_solver(num_agents_ > 1);
}

void PathSyncApp::select_solver_by_index(std::size_t index)
{
    path_finder_.select_solver_by_index(index);
}

void PathSyncApp::select_solver_by_index(std::size_t index, bool multi_agent)
{
    if (multi_agent)
        path_finder_.select_ma_solver_by_index(index);
    else
        path_finder_.select_sa_solver_by_index(index);
}

std::vector<std::string> PathSyncApp::get_solver_names() const
{
    return path_finder_.get_all_solver_names();
}

std::vector<std::string> PathSyncApp::get_solver_names(bool multi_agent) const
{
    if (multi_agent)
        return path_finder_.get_ma_solver_names();
    return path_finder_.get_sa_solver_names();
}

bool PathSyncApp::get_is_multi_agent() const
{
    return num_agents_ > 1;
}

void PathSyncApp::toggle_agent_mode()
{
    for (auto &elem : current_scene_.first)
        current_map_data_->set_cell_type(elem, path_sync::CellType::DEFAULT);

    for (auto &elem : current_scene_.second)
        current_map_data_->set_cell_type(elem, path_sync::CellType::DEFAULT);

    int max_agents = std::min(10, map_manager_.get_total_scenes());
    num_agents_ = (num_agents_ % max_agents) + 1;

    map_manager_.reset_scene_index();
    current_scene_ = map_manager_.get_next_scene(num_agents_);
    update_map_data_with_current_scene_();

    std::stringstream ss;
    ss << "Switched to " << num_agents_ << "-agent mode.";
    path_sync::Logger::get().info(ss.str().c_str());
}

std::shared_ptr<path_sync::MapData> PathSyncApp::get_current_map_data() const
{
    return current_map_data_;
}

std::string_view PathSyncApp::get_current_solver_name() const
{
    return path_finder_.get_current_solver_name();
}

bool PathSyncApp::is_solver_optimal(std::size_t index, bool multi_agent) const
{
    if (multi_agent)
        return path_finder_.is_ma_solver_optimal(index);
    return path_finder_.is_sa_solver_optimal(index);
}

int PathSyncApp::get_scene_index() const
{
    return map_manager_.get_current_scene_index();
}

int PathSyncApp::get_total_scenes() const
{
    return map_manager_.get_total_scenes();
}

int PathSyncApp::get_map_index() const
{
    return map_manager_.get_current_map_index();
}

int PathSyncApp::get_total_maps() const
{
    return map_manager_.get_total_maps();
}

std::string PathSyncApp::get_current_map_name() const
{
    return current_map_data_->get_map_info().map_name;
}

int PathSyncApp::get_num_agents() const { return num_agents_; }

PerformanceMetrics PathSyncApp::get_performance_metrics() const
{
    return path_finder_.get_performance_metrics();
}

void PathSyncApp::clear_scene()
{
    for (int y = 0; y < current_map_data_->get_height(); ++y)
    {
        for (int x = 0; x < current_map_data_->get_width(); ++x)
        {
            CellType current_type = current_map_data_->get_cell_type(Coordinate(x, y));
            if (current_type == path_sync::CellType::START || current_type == path_sync::CellType::END)
            {
                current_map_data_->set_cell_type(Coordinate(x, y), CellType::DEFAULT);
            }
        }
    }
    path_sync::Logger::get().info("Scene cleared.");
}

void PathSyncApp::clear_paths()
{
    for (int y = 0; y < current_map_data_->get_height(); ++y)
    {
        for (int x = 0; x < current_map_data_->get_width(); ++x)
        {
            CellType current_type = current_map_data_->get_cell_type(Coordinate(x, y));
            if (current_type == CellType::PATH || current_type == CellType::VISITED || current_type == CellType::FOUND)
            {
                current_map_data_->set_cell_type(Coordinate(x, y), CellType::DEFAULT);
            }
        }
    }
    path_sync::Logger::get().info("Paths cleared.");
}

void PathSyncApp::reset_grid()
{
    current_map_data_ = std::make_shared<path_sync::MapData>(map_manager_.get_current_map_data());
    current_scene_.first.clear();
    current_scene_.second.clear();
    current_sa_solution_.clear();
    current_ma_solution_.clear();
    path_sync::Logger::get().info("Grid reset.");
}

void PathSyncApp::update_map_data_with_current_scene_()
{
    for (auto &elem : current_scene_.first)
        current_map_data_->set_cell_type(elem, path_sync::CellType::START);

    for (auto &elem : current_scene_.second)
        current_map_data_->set_cell_type(elem, path_sync::CellType::END);
}

} // namespace path_sync
