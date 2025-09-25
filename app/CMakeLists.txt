add_executable(path_sync_app
    src/main.cpp
    src/PathSyncApp.cpp
)

target_include_directories(path_sync_app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/libs/path_sync_core/include
    ${CMAKE_SOURCE_DIR}/libs/path_sync_ui/include
)

target_link_libraries(path_sync_app PUBLIC
    path_sync_core
)

