cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(path_sync
    DESCRIPTION "Visualizing and Analyzing Path Finding Algorithm."
    VERSION 0.0.2
    LANGUAGES CXX
)

add_subdirectory(libs/path_sync_core)
add_subdirectory(libs/path_sync_ui)
add_subdirectory(app)
add_subdirectory(test)

add_executable(path_sync 
    ./main.cpp
    ./app/src/PathSyncApp.cpp
)

target_include_directories(path_sync PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/app/src
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/path_sync_core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libs/path_sync_ui/include
)

target_link_libraries(path_sync PUBLIC
    path_sync_core
    path_sync_ui
)
