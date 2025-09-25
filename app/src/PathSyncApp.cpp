#include "PathSyncApp.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace path_sync
{

PathSyncApp::PathSyncApp(int width, int height)
    : map_loader_{}
    , map_data_(width, height)
    , path_finder_{}
    , current_map_width_(width)
    , current_map_height_(height)
{
}

void PathSyncApp::run()
{
    path_sync::Logger::get().info("PathSyncApp running in UI mode...");
    // In UI mode, the VisualizationSystem drives the loop.
    // This method might be used for background tasks or initial setup.
}

void PathSyncApp::solve()
{
    if (start_points_.empty() || end_points_.empty())
    {
        path_sync::Logger::get().warn("No start or end points defined.");
        return;
    }

    // Clear previous paths in map_data_
    clear_paths();

    // Set start and end points in map_data_ for visualization
    for (const auto &p : start_points_)
    {
        map_data_.set_cell_type(p.first, p.second, CellType::START);
    }

    for (const auto &p : end_points_)
    {
        map_data_.set_cell_type(p.first, p.second, CellType::END);
    }

    // Call the pathfinder
    std::variant<std::vector<Coordinate>, std::vector<std::vector<Coordinate>>> result =
        path_finder_.find_path(map_data_, start_points_, end_points_);

    if (std::holds_alternative<std::vector<Coordinate>>(result))
    {
        path_sync::Logger::get().info("Found a single-agent solution.");
        std::vector<Coordinate> path = std::get<std::vector<Coordinate>>(result);
        for (const auto &coord : path)
        {
            map_data_.set_cell_type(coord.first, coord.second, CellType::PATH);
        }
    }
    else if (std::holds_alternative<std::vector<std::vector<Coordinate>>>(result))
    {
        path_sync::Logger::get().info("Found a multi-agent solution.");
        std::vector<std::vector<Coordinate>> paths = std::get<std::vector<std::vector<Coordinate>>>(result);
        for (const auto &path : paths)
        {
            for (const auto &coord : path)
            {
                map_data_.set_cell_type(coord.first, coord.second, CellType::PATH);
            }
        }
    }
    path_sync::Logger::get().info(get_performance_data().str().c_str());
}

void PathSyncApp::run_headless_experiments()
{
    path_sync::Logger::get().info("Running headless experiments...");
    // Example headless experiment: load a map, a scenario, solve, and log.
    // This would typically involve iterating through multiple maps/scenarios/solvers.

    load_map("arena2"); // Load a specific map
    load_scenario(1);   // Load a scenario for 2 agents
    solve();            // Solve the problem
    path_sync::Logger::get().info(get_performance_data().str().c_str());

    // More complex logic would go here: loops, different solver selections, etc.
}

void PathSyncApp::load_map(const std::string &map_name)
{
    map_loader_ = path_sync::Map(map_name);
    const path_sync::MapInfo &map_info = map_loader_.get_map_info();

    current_map_width_ = map_info.width;
    current_map_height_ = map_info.height;
    map_data_ = path_sync::MapData(current_map_width_, current_map_height_);

    std::string line;
    std::stringstream map_stream;
    map_stream << map_info.map.str();

    int y = 0;
    while (std::getline(map_stream, line))
    {
        for (int x = 0; x < line.length(); ++x)
        {
            if (line[x] == '@' || line[x] == 'O' || line[x] == 'T' || line[x] == 'W')
            {
                map_data_.set_cell_type(x, y, CellType::WALL);
            }
            else
            {
                map_data_.set_cell_type(x, y, CellType::DEFAULT);
            }
        }
        y++;
    }
    path_sync::Logger::get().info(("Map '" + map_name + "' loaded.").c_str());
    clear_paths(); // Clear any old paths/points
}

void PathSyncApp::load_scenario(int num_agents)
{
    std::pair<std::vector<Coordinate>, std::vector<Coordinate>> agents_data = map_loader_.load_n_agents(num_agents);
    start_points_ = agents_data.first;
    end_points_ = agents_data.second;

    // Update map_data_ with start/end points for visualization
    for (const auto &p : start_points_)
    {
        map_data_.set_cell_type(p.first, p.second, CellType::START);
    }
    for (const auto &p : end_points_)
    {
        map_data_.set_cell_type(p.first, p.second, CellType::END);
    }

    path_sync::Logger::get().info(("Scenario for " + std::to_string(num_agents) + " agents loaded.").c_str());
}

void PathSyncApp::add_start_point(Coordinate point)
{
    start_points_.push_back(point);
    map_data_.set_cell_type(point.first, point.second, CellType::START);
}

void PathSyncApp::add_end_point(Coordinate point)
{
    end_points_.push_back(point);
    map_data_.set_cell_type(point.first, point.second, CellType::END);
}

void PathSyncApp::clear_paths()
{
    for (int y = 0; y < map_data_.get_height(); ++y)
    {
        for (int x = 0; x < map_data_.get_width(); ++x)
        {
            CellType current_type = map_data_.get_cell_type(x, y);
            if (current_type == CellType::PATH || current_type == CellType::VISITED || current_type == CellType::FOUND)
            {
                map_data_.set_cell_type(x, y, CellType::DEFAULT);
            }
        }
    }
    path_sync::Logger::get().info("Paths cleared.");
}

void PathSyncApp::reset_grid()
{
    map_data_ = path_sync::MapData(current_map_width_, current_map_height_); // Recreate map_data
    start_points_.clear();
    end_points_.clear();
    path_sync::Logger::get().info("Grid reset.");
}

void PathSyncApp::change_map()
{
    path_sync::MapInfo &new_map_info = map_loader_.change_current_map();
    load_map(new_map_info.map_name);
    path_sync::Logger::get().info(("Changed map to: " + new_map_info.map_name).c_str());
}

void PathSyncApp::change_solver()
{
    path_finder_.change_solver();
    path_sync::Logger::get().info(
        ("Changed solver to: " + std::string(path_finder_.get_current_solver_name())).c_str());
}

} // namespace path_sync
