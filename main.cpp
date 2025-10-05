#include "app/src/PathSyncApp.hpp"
#include "path_sync_ui/visualization_system.hpp"
#include "path_sync_ui/visualization_system_config.hpp"
#include "path_sync_core/logger.hpp"

int path_sync::VisualizationSystemConfig::CELL_SIZE = 5;

int main()
{
    // path_sync::PathSyncApp app_;
    // std::unique_ptr<path_sync::VisualizationSystemConfig> vsc_ = 
    //     std::make_unique<path_sync::VisualizationSystemConfig>(std::string(PROJECT_ROOT) + "/config/env_vars.yaml");
    //
    // path_sync::VisualizationSystem visualization_system_(app_, std::move(vsc_));
    // visualization_system_.run();
    path_sync::Logger::get().set_log_file_path(std::string(PROJECT_ROOT) + "/log/logging.txt");
    path_sync::Logger::get().info("Testing if logging file works");
}
