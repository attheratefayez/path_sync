cmake_policy(SET CMP0135 NEW)

include(FetchContent)
FetchContent_Declare(
    google_test
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(google_test)

set(test_src_files
    ./main_test.cpp
    ./path_sync_core_test/map_scene_test.cpp
    ./path_sync_core_test/env_map_test.cpp
    ./path_sync_core_test/map_data_test.cpp
    ./path_sync_core_test/map_manager_test.cpp
    ./path_sync_core_test/solver_test.cpp
    ./path_sync_core_test/performance_metrics_test.cpp
    ./path_sync_ui_test/cell_test.cpp
    ./path_sync_ui_test/color_test.cpp
    ./path_sync_ui_test/grid_test.cpp
    ${CMAKE_SOURCE_DIR}/app/src/PathSyncApp.cpp
)

enable_testing()
add_executable(path_sync_test
    ${test_src_files}
)

target_include_directories(path_sync_test PUBLIC
    ${CMAKE_SOURCE_DIR}/libs/path_sync_ui/include
    ${CMAKE_SOURCE_DIR}/app/src
)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

target_link_libraries(path_sync_test
    GTest::gtest_main
    Qt6::Widgets
    yaml-cpp
    path_sync_core
    path_sync_ui
)

include(GoogleTest)
gtest_discover_tests(path_sync_test)
