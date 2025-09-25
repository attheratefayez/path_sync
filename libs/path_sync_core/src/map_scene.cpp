#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>

#include "path_sync_core/logger.hpp"
#include "path_sync_core/map_loader/map_scene.hpp"
#include "path_sync_core/path_sync_types.hpp"

namespace path_sync
{
Scene::Scene(std::string map_name) : map_name_(map_name)
{
    std::string folder_name = map_name + ".map-scen";
    std::string scene_file_name = map_name + ".map.scen";

    std::filesystem::path scene_file_path =
        std::string(PROJECT_ROOT) + std::string("/maps/scenes/") + folder_name + "/" + scene_file_name;

    read_scene_(scene_file_name);
}

Scene::Scene(Scene &&map_scene)
{
    map_name_ = std::move(map_scene.map_name_);
    agent_pool_ = std::move(map_scene.agent_pool_);
    bucket_agent_mapping_ = std::move(map_scene.bucket_agent_mapping_);
}

Scene &Scene::operator=(Scene &&map_scene)
{
    *this = std::move(map_scene);
    return *this;
}

void Scene::read_scene_(std::filesystem::path scene_file_path)
{
    if (std::filesystem::exists(scene_file_path))
    {
        std::stringstream ss;
        std::ifstream scene_file(scene_file_path);

        while (scene_file.is_open())
        {

            std::string temp;
            scene_file >> temp;
            ss << temp << " ";
            scene_file >> temp;
            ss << temp << "\n";

            int bucket, startx, starty, goalx, goaly;
            std::string line;

            while (std::getline(scene_file, line))
            {
                std::stringstream sline(line);
                sline >> bucket >> temp >> temp >> temp >> startx >> starty >> goalx >> goaly >> temp;

                if (!bucket_agent_mapping_.count(bucket))
                    bucket_agent_mapping_[bucket] = std::pair<std::vector<Coordinate>, std::vector<Coordinate>>();

                Coordinate start_pos(startx, starty);
                Coordinate goal_pos(goalx, goaly);

                bucket_agent_mapping_[bucket].first.push_back(start_pos);
                bucket_agent_mapping_[bucket].second.push_back(goal_pos);
                agent_pool_.push_back(std::pair<Coordinate, Coordinate>(start_pos, goal_pos));
            }
            scene_file.close();
        }
    }

    else
    {
        throw std::filesystem::filesystem_error("Scene file does not exist or is not a regular file", scene_file_path,
                                                std::make_error_code(std::errc::no_such_file_or_directory));
    }
}

} // namespace path_sync
