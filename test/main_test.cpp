#include <gtest/gtest.h>

#include "path_sync/visualization_system/visualization_system_config.hpp"
int path_sync::VisualizationSystemConfig::CELL_SIZE = 50;

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
