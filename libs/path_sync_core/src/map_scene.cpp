#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include "path_sync_core/logger.hpp"
#include "path_sync_core/map_loader/map_scene.hpp"
#include "path_sync_core/path_sync_types.hpp"

namespace path_sync
{

Scene::Scene(std::string map_name)
    : map_name_(map_name)
{
    std::string folder_ext = "-scen";
    std::string file_ext = ".scen";

    if (map_name.find(".map") == std::string::npos)
    {
        folder_ext = ".map" + folder_ext;
        file_ext = ".map" + file_ext;
    }

    std::string folder_name = map_name + folder_ext;
    std::string scene_file_name = map_name + file_ext;

    std::filesystem::path scene_file_path =
        std::string(PROJECT_ROOT) + std::string("/maps/scenes/") + folder_name + "/" + scene_file_name;

    read_scene_(scene_file_path);
}

Scene::Scene(Scene &&map_scene)
{
    map_name_ = std::move(map_scene.map_name_);
    scene_data_ = std::move(map_scene.scene_data_);
}

Scene &Scene::operator=(Scene &&map_scene)
{
    if (this ==  &map_scene)
        return *this;

    this->map_name_ = std::move(map_scene.map_name_);
    this->scene_data_ = std::move(map_scene.scene_data_);

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
            std::getline(scene_file, temp); // ignoring first line

            int startx, starty, goalx, goaly;
            std::string line;

            while (std::getline(scene_file, line))
            {
                std::stringstream sline(line);
                sline >> temp >> temp >> temp >> temp >> startx >> starty >> goalx >> goaly >> temp;

                Coordinate start_pos(startx, starty);
                Coordinate goal_pos(goalx, goaly);

                scene_data_.push_back(std::pair<Coordinate, Coordinate>(start_pos, goal_pos));
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
