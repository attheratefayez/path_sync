#include <filesystem>
#include <fstream>
#include <string>

#include "path_sync_core/map_loader/env_map.hpp"

namespace path_sync
{

Map::Map(std::string mapname)
{
    std::string map_ext = "";
    std::string::size_type n = mapname.find(".map");

    if (n == std::string::npos)
        map_ext = ".map";

    map_info_.map_name = mapname + map_ext;

    std::filesystem::path map_path = std::string(PROJECT_ROOT) + std::string("/maps/") + map_info_.map_name;

    read_map_(map_path);
    map_scenes_ = std::move(Scene(map_info_.map_name));
}

Map::Map(Map &&other)
{
    if(this == &other)
        return;

    this->map_info_ = std::move(other.map_info_);
    this->map_scenes_ = std::move(other.map_scenes_);
}

Map &Map::operator=(Map &&other)
{
    if(this == &other)
        return *this;

    this->map_info_ = std::move(other.map_info_);
    this->map_scenes_ = std::move(other.map_scenes_);

    return *this;
}

void Map::read_map_(std::filesystem::path map_path)
{
    if (std::filesystem::exists(map_path))
    {
        std::ifstream map_file(map_path);
        std::string temp_str;

        while (map_file.is_open())
        {
            map_file >> temp_str >> temp_str >> temp_str >> map_info_.height >> temp_str >> map_info_.width >> temp_str;

            while (std::getline(map_file, temp_str))
            {
                if (!temp_str.size() or temp_str.size() != map_info_.width)
                    continue;

                map_info_.map << temp_str << "\n";
            }
            map_file.close();
        }
    }

    else
    {
        throw std::filesystem::filesystem_error("Map file does not exist or is not a regular file", map_path,
                                                std::make_error_code(std::errc::no_such_file_or_directory));
    }
}

} // namespace path_sync
