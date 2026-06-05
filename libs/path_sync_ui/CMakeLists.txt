file(GLOB path_sync_ui_srcs "./src/*.cpp")
file(GLOB path_sync_ui_headers "./include/path_sync_ui/*.hpp")

add_library(path_sync_ui SHARED
    ${path_sync_ui_srcs}
    ${path_sync_ui_headers}
)

set_target_properties(path_sync_ui PROPERTIES
    AUTOMOC ON
)

target_include_directories(path_sync_ui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/libs/path_sync_core/include
    ${CMAKE_SOURCE_DIR}/app/src
)

find_package(Qt6 REQUIRED COMPONENTS Widgets Concurrent)

target_link_libraries(path_sync_ui PUBLIC
    Qt6::Widgets
    Qt6::Concurrent
    path_sync_core
    yaml-cpp
)
