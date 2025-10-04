file(GLOB path_sync_ui_srcs "./src/*.cpp")

add_library(path_sync_ui SHARED
    ${path_sync_ui_srcs}
)

target_include_directories(path_sync_ui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/libs/path_sync_core/include
    ${CMAKE_SOURCE_DIR}/app/src
)

target_link_directories(path_sync_ui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/SFML
)

target_link_libraries(path_sync_ui PUBLIC
    sfml-graphics
    sfml-system
    sfml-window
    path_sync_core
    yaml-cpp
)
