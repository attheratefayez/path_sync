#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "path_sync/map_loader/env_map.hpp"

namespace psync
{

Map::Map(std::string mapname) : __map_name(mapname), __map_scenes(Scene(mapname)), __current_bucket(0)
{
    std::filesystem::path map_path = std::string(PROJECT_ROOT) + std::string("/maps/") + __map_name + ".map";

    if (std::filesystem::exists(map_path))
    {
        std::ifstream map_file(map_path);
        std::string temp_str;

        while (map_file.is_open())
        {
            map_file >> temp_str >> temp_str >> temp_str >> __map_height >> temp_str >> __map_width >> temp_str;

            while (std::getline(map_file, temp_str))
            {
                if (!temp_str.size())
                    continue;

                __map << temp_str << "\n";
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

Map::Map(Map &map_obj)
{
    std::stringstream ss(map_obj.__map.str());

    __map_height = map_obj.__map_height;
    __map_width = map_obj.__map_width;
    __map << ss.str();
    __map_name = map_obj.__map_name;
    __map_scenes = map_obj.__map_scenes;
    __current_bucket = map_obj.__current_bucket;
}

Map& Map::operator=(const Map& obj)
{
    std::stringstream ss(obj.__map.str());

    __map_height = obj.__map_height;
    __map_width = obj.__map_width;
    __map.str(std::string());
    __map.str(ss.str());
    __map_name = obj.__map_name;
    __map_scenes = obj.__map_scenes;
    __current_bucket = obj.__current_bucket;

    return *this;
}

}


