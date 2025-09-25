#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "path_sync_core/map_loader/env_map.hpp"
#include "path_sync_core/logger.hpp"

namespace path_sync
{

Map::Map(std::string mapname) : selected_map_index_(0), current_bucket_idx_(0), available_agents_in_current_bucket_(0)
{
    get_available_maps_();
    current_map_info_.map_name = available_maps_[selected_map_index_];

    std::filesystem::path map_path =
        std::string(PROJECT_ROOT) + std::string("/maps/") + current_map_info_.map_name + ".map";

    read_map_(map_path);
    map_scenes_ = std::move(Scene(current_map_info_.map_name));
}

Map::Map(Map &&other)
{
    available_maps_ = std::move(other.available_maps_);
    selected_map_index_ = std::move(other.selected_map_index_);
    current_map_info_ = std::move(other.current_map_info_);
    map_scenes_ = std::move(other.map_scenes_);
    current_bucket_idx_ = std::move(other.current_bucket_idx_);
    current_bucket_ = std::move(other.current_bucket_);
}

Map &Map::operator=(Map &&other)
{
    *this = std::move(other);
    return *this;
}

std::pair<std::vector<Coordinate>, std::vector<Coordinate>> Map::load_n_agents(int n)
{
    if (!current_bucket_.first.size())
        get_current_bucket_();

    if (n > 10)
    {
        std::stringstream ss;
        ss << "I am not going to give you more then 10 agents at a time.\n";
        ss << "If you still need it, change the logic, compile and get your stuff.\n";
        n %= 11;
        n = n ? n : 1;

        path_sync::Logger::get().warn(ss.str().c_str());
    }

    int count = 0;
    std::pair<std::vector<Coordinate>, std::vector<Coordinate>> ret_val;
    std::pair<std::vector<Coordinate>, std::vector<Coordinate>> extras;

    if (n > available_agents_in_current_bucket_)
    {
        extras = {{current_bucket_.first.end() - available_agents_in_current_bucket_, current_bucket_.first.end()},
                  {current_bucket_.second.end() - available_agents_in_current_bucket_, current_bucket_.second.end()}};
        count = available_agents_in_current_bucket_;
        Map::next_bucket_();
    }

    ret_val = {{current_bucket_.first.begin(), current_bucket_.first.begin() + n - count},
               {current_bucket_.second.begin(), current_bucket_.second.begin() + n - count}};
    available_agents_in_current_bucket_ = n - count;

    if (extras.first.size())
    {
        for (int i = 0; i < extras.first.size(); ++i)
        {
            ret_val.first.push_back(extras.first[i]);
            ret_val.second.push_back(extras.second[i]);
        }
    }

    return ret_val;
}


MapInfo& Map::change_current_map()
{
    ++selected_map_index_;
    if(selected_map_index_ >= available_maps_.size())
    {
        std::stringstream ss;
        ss << "No more maps.\n";

        path_sync::Logger::get().warn(ss.str().c_str());
        selected_map_index_ %= available_maps_.size();
    }

    current_map_info_.map_name = available_maps_[selected_map_index_];
    std::filesystem::path map_path =
        std::string(PROJECT_ROOT) + std::string("/maps/") + current_map_info_.map_name + ".map";
    read_map_(map_path);
    map_scenes_ = std::move(Scene(map_path));

    return current_map_info_;
}

std::pair<std::vector<Coordinate>, std::vector<Coordinate>> Map::get_current_bucket_()
{
    std::variant<std::pair<std::vector<Coordinate>, std::vector<Coordinate>>, int> maybe_agent_pool =
        map_scenes_.get_nth_bucket(current_bucket_idx_);

    if (std::holds_alternative<int>(maybe_agent_pool))
    {
        std::stringstream ss;
        ss << "No more buckets in current map.\n";
        ss << "Getting the first bucket.\n";
        path_sync::Logger::get().warn(ss.str().c_str());

        current_bucket_idx_ %= std::get<int>(maybe_agent_pool);
        maybe_agent_pool = map_scenes_.get_nth_bucket(current_bucket_idx_);
    }

    current_bucket_ = std::get<std::pair<std::vector<Coordinate>, std::vector<Coordinate>>>(maybe_agent_pool);
    available_agents_in_current_bucket_ = current_bucket_.first.size();

    return current_bucket_;
}

std::pair<std::vector<Coordinate>, std::vector<Coordinate>> Map::next_bucket_()
{
    ++current_bucket_idx_;

    return Map::get_current_bucket_();
}

std::pair<std::vector<Coordinate>, std::vector<Coordinate>> Map::previous_bucket_()
{
    --current_bucket_idx_;
    if (current_bucket_idx_ < 0)
    {
        path_sync::Logger::get().warn("No previous bucket.");
        current_bucket_idx_ = 0;
    }
    return Map::get_current_bucket_();
}

void Map::get_available_maps_()
{
    std::string map_directory = std::string(PROJECT_ROOT) + "/maps/";

    for (const auto &entry : std::filesystem::directory_iterator(map_directory))
    {
        if (std::filesystem::is_regular_file(entry))
        {
            available_maps_.push_back(entry.path().filename().stem());
        }
    }
}

void Map::read_map_(std::filesystem::path map_path)
{
    if (std::filesystem::exists(map_path))
    {
        std::ifstream map_file(map_path);
        std::string temp_str;

        while (map_file.is_open())
        {
            map_file >> temp_str >> temp_str >> temp_str >> current_map_info_.height >> temp_str >>
                current_map_info_.width >> temp_str;

            while (std::getline(map_file, temp_str))
            {
                if (!temp_str.size())
                    continue;

                current_map_info_.map << temp_str << "\n";
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
