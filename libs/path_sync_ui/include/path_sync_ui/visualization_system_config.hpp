/**
 * @file visualization_system_config.hpp
 * @brief VisualizationSystemConfig struct for storing system env_vars.
 */

#ifndef __PATH_SYNC_VISUALIZATION_SYSTEM_CONFIG_HPP__
#define __PATH_SYNC_VISUALIZATION_SYSTEM_CONFIG_HPP__

#include <filesystem>
#include <string>
#include <yaml-cpp/yaml.h>

namespace path_sync
{
/**
 * @class VisualizationSystemConfig
 * @brief PathSync Visulaization system's env_vars.
 *
 * Reads a yaml file to load the env_vars.
 *
 */
struct VisualizationSystemConfig
{
    std::string TITLE;

    unsigned int WIDTH;
    unsigned int HEIGHT;
    unsigned int FRAMERATE;

    static int CELL_SIZE;

    VisualizationSystemConfig() = default;

    explicit VisualizationSystemConfig(std::filesystem::path config_file_path)
    {
        YAML::Node config = YAML::LoadFile(config_file_path);

        TITLE = config["window"]["title"].as<std::string>();
        WIDTH = config["window"]["width"].as<int>();
        HEIGHT = config["window"]["height"].as<int>();
        FRAMERATE = config["window"]["framerate"].as<int>();
    }
}; // end of struct VisualizationSystemConfig

} // end of namespace path_sync

#endif // !__PATH_SYNC_VISUALIZATION_SYSTEM_CONFIG_HPP__
