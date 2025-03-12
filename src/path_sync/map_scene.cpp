#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>

#include "path_sync/logger.hpp"
#include "path_sync/psync_types.hpp"
#include "path_sync/map_scene.hpp"

namespace psync
{
Scene::Scene(std::string mapname) : map_name(mapname)
{
    std::string folder_name = map_name + ".map-scen";
    std::string scene_file_name = map_name + ".map.scen";
    std::filesystem::path scene_file_path = std::string(PROJECT_ROOT) + std::string("/maps/scenes/") + folder_name + "/" + scene_file_name;

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

                if (!bucket_agent_mapping.count(bucket))
                    bucket_agent_mapping[bucket] = std::pair<std::vector<Coordinate>, std::vector<Coordinate>>();

                Coordinate start_pos(startx, starty);
                Coordinate goal_pos(goalx, goaly);

                bucket_agent_mapping[bucket].first.push_back(start_pos);
                bucket_agent_mapping[bucket].second.push_back(goal_pos);
                agent_pool.push_back(std::pair<Coordinate, Coordinate>(start_pos, goal_pos));
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

std::vector<std::pair<Coordinate, Coordinate>> Scene::get_n_agent(int no_of_agenets)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(agent_pool.begin(), agent_pool.end(), gen);
    std::vector<std::pair<Coordinate, Coordinate>> ret;

    if (agent_pool.size() >= no_of_agenets)
    {

        ret = std::vector<std::pair<Coordinate, Coordinate>>(agent_pool.begin(), agent_pool.begin() + no_of_agenets);

        agent_pool = std::vector<std::pair<Coordinate, Coordinate>>(agent_pool.begin() + no_of_agenets, agent_pool.end());
    }

    else
    {
        psync::Logger::get()->warn("Dont have enough agents");
    }

    return ret;
}
}

