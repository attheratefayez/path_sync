#include <filesystem>
#include <stdexcept>

#include "path_sync_core/logger.hpp"
#include "path_sync_core/map_loader/map_manager.hpp"
#include "path_sync_core/path_sync_types.hpp"

namespace path_sync
{

MapManager::MapManager()
    : current_map_{}
    , current_map_data_{}
    , current_scene_{}
    , current_map_idx_{-1}
    , current_scene_idx_{0}
    , total_scenes_{}
{
    _get_available_maps();
}

[[nodiscard]] path_sync::MapData MapManager::get_current_map_data()
{
    if (current_map_.get_map_scenes().get_scene_data().empty())
    {
        throw std::runtime_error(
            "No map-data available. Map is not initialied. Call get_next_map_data() to load a map.");
    }

    return current_map_data_;
}

[[nodiscard]] path_sync::MapData MapManager::get_next_map_data()
{
    if (current_map_idx_ + 1 < available_maps_.size())
    {
        ++current_map_idx_;

        current_map_ = std::move(Map(available_maps_[current_map_idx_]));
        current_map_data_ = MapData(current_map_.get_map_info());

        current_scene_.first.clear();
        current_scene_.second.clear();
        current_scene_idx_ = 0;

        total_scenes_ = current_map_.get_map_scenes().get_scene_data().size();

        return current_map_data_;
    }

    return MapData();
}

std::pair<std::vector<Coordinate>, std::vector<Coordinate>> MapManager::get_current_scene() const
{
    if (current_scene_.first.empty())
    {
        throw std::runtime_error("No scene available. Map is not initialied. Call get_next_map_data() to load a map.");
    }

    return current_scene_;
}

[[nodiscard]] std::pair<std::vector<Coordinate>, std::vector<Coordinate>> MapManager::get_next_scene(int n_agent)
{
    // NOTE:
    // Scene data is returned as a vector of start-end pairs.
    // But, when user requests for a scene, MapManager should return
    // scene as a pair of all start vectors and all end vectors.
    // Because, PathFinder will expect a vector of starts and a vector of ends.

    if (current_map_.get_map_scenes().get_scene_data().empty())
    {
        throw std::runtime_error("No scene available. Map is not initialied. Call get_next_map_data() to load a map.");
    }

    if (current_scene_idx_ + n_agent < total_scenes_)
    {
        std::vector<std::pair<path_sync::Coordinate, path_sync::Coordinate>> temp(
            current_map_.get_map_scenes().get_scene_data().begin() + current_scene_idx_,
            current_map_.get_map_scenes().get_scene_data().begin() + current_scene_idx_ + n_agent);

        current_scene_.first.reserve(n_agent);
        current_scene_.second.reserve(n_agent);
        current_scene_.first.clear();
        current_scene_.second.clear();

        for (auto start_end_pair : temp)
        {
            current_scene_.first.push_back(start_end_pair.first);
            current_scene_.second.push_back(start_end_pair.second);
        }

        current_scene_idx_ += n_agent;

        std::stringstream ss;
        ss << "Scene: " << current_scene_idx_+1 << " Total Scenes: " << total_scenes_ << std::endl;
        path_sync::Logger::get().info(ss.str().c_str());

        return current_scene_;
    }

    return std::pair<std::vector<Coordinate>, std::vector<Coordinate>>();
}

void MapManager::reset()
{
    *this = MapManager();
}

void MapManager::_get_available_maps()
{
    std::string map_directory = std::string(PROJECT_ROOT) + "/maps/";

    for (const auto &entry : std::filesystem::directory_iterator(map_directory))
    {
        if (std::filesystem::is_regular_file(entry))
        {
            available_maps_.push_back(entry.path().filename().stem());
        }
    }

    available_maps_.shrink_to_fit();
}

} // namespace path_sync
