#include "PathSyncApp.hpp"
#include "path_sync_core/logger.hpp"
#include "path_sync_core/path_sync_types.hpp"

#include <iostream>
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
    current_map_data_ = map_manager_.get_next_map_data();
    current_scene_ = map_manager_.get_next_scene(num_agents_);

    update_map_data_with_current_scene_();

    std::stringstream ss;
    ss << "Current map: " << map_manager_.get_current_map_data().get_map_info().map_name << std::endl;

    path_sync::Logger::get().info(ss.str().c_str());
}

bool PathSyncApp::request_next_map()
{
    current_map_data_ = map_manager_.get_next_map_data();
    return bool(current_map_data_.get_height());
}

bool PathSyncApp::request_next_scene()
{
    clear_scene();
    current_scene_ = map_manager_.get_next_scene(num_agents_);
    update_map_data_with_current_scene_();
    return bool(current_scene_.first.size());
}

bool PathSyncApp::solve_current_scene()
{
    auto current_solution_ = path_finder_.find_path(current_map_data_, current_scene_.first, current_scene_.second);

    if (std::holds_alternative<std::vector<Coordinate>>(current_solution_))
    {
        current_sa_solution_ = std::get<std::vector<Coordinate>>(current_solution_);
        if (current_sa_solution_.empty())
            return false;

        for (Coordinate const &elem : current_sa_solution_)
            current_map_data_.set_cell_type(elem, path_sync::CellType::PATH);
    }
    else
    {
        current_ma_solution_ = std::get<std::vector<std::vector<Coordinate>>>(current_solution_);
        if (std::all_of(current_ma_solution_.begin(), current_ma_solution_.end(),
                        [](std::vector<Coordinate> &elem) { return not elem.empty(); }))
            return false;

        for (auto &path : current_ma_solution_)
        {
            for (auto &elem : path)
                current_map_data_.set_cell_type(elem, path_sync::CellType::PATH);
        }
    }

    Logger::get().info(path_finder_.get_performance_data().str().c_str());
    return true;
}

std::shared_ptr<path_sync::MapData const> PathSyncApp::get_current_map_data() const
{
    std::shared_ptr<path_sync::MapData const> temp = std::make_shared<path_sync::MapData const>(current_map_data_);
    return temp;
}

void PathSyncApp::clear_scene()
{
    for (int y = 0; y < current_map_data_.get_height(); ++y)
    {
        for (int x = 0; x < current_map_data_.get_width(); ++x)
        {
            CellType current_type = current_map_data_.get_cell_type(Coordinate(x, y));
            if (current_type == path_sync::CellType::START || current_type == path_sync::CellType::END)
            {
                current_map_data_.set_cell_type(Coordinate(x, y), CellType::DEFAULT);
            }
        }
    }
    path_sync::Logger::get().info("Scene cleared.");
}


void PathSyncApp::clear_paths()
{
    for (int y = 0; y < current_map_data_.get_height(); ++y)
    {
        for (int x = 0; x < current_map_data_.get_width(); ++x)
        {
            CellType current_type = current_map_data_.get_cell_type(Coordinate(x, y));
            if (current_type == CellType::PATH || current_type == CellType::VISITED || current_type == CellType::FOUND)
            {
                current_map_data_.set_cell_type(Coordinate(x, y), CellType::DEFAULT);
            }
        }
    }
    path_sync::Logger::get().info("Paths cleared.");
}

void PathSyncApp::reset_grid()
{
    current_map_data_ = map_manager_.get_current_map_data();
    current_scene_.first.clear();
    current_scene_.second.clear();
    current_sa_solution_.clear();
    current_ma_solution_.clear();
    path_sync::Logger::get().info("Grid reset.");
}

void PathSyncApp::update_map_data_with_current_scene_()
{
    for(auto& elem: current_scene_.first)
    current_map_data_.set_cell_type(elem, path_sync::CellType::START);

    for(auto& elem: current_scene_.second)
    current_map_data_.set_cell_type(elem, path_sync::CellType::END);
}

} // namespace path_sync
