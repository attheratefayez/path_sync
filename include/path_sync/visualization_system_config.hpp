/**
 * @file visualization_system_config.hpp
 * @brief VisualizationSystemConfig struct for storing system env_vars.
 */

#ifndef __PATH_SYNC_VISUALIZATION_SYSTEM_CONFIG_HPP__
#define __PATH_SYNC_VISUALIZATION_SYSTEM_CONFIG_HPP__

#include <filesystem>
#include <string>
#include <yaml-cpp/yaml.h>

namespace psync{
/**
 * @class VisualizationSystemConfig
 * @brief PathSync Visulaization system's env_vars. 
 *
 * Reads a yaml file to load the env_vars.
 *
 */
struct VisualizationSystemConfig 
{
    unsigned int WIDTH;
    unsigned int HEIGHT;
    unsigned int FRAMERATE;
    std::string TITLE;
    static int CELL_SIZE;

    VisualizationSystemConfig(std::filesystem::path config_file_path)
    {
        YAML::Node config = YAML::LoadFile(config_file_path);    
        WIDTH = config["window"]["width"].as<int>();
        HEIGHT = config["window"]["height"].as<int>();
        FRAMERATE = config["window"]["framerate"].as<int>();
        TITLE = config["window"]["title"].as<std::string>();
    }
};
}


#endif // !__PATH_SYNC_VISUALIZATION_SYSTEM_CONFIG_HPP__
