set(CMAKE_AUTOMOC ON)

file(GLOB path_sync_ui_srcs "./src/*.cpp")

add_library(path_sync_ui SHARED
    ${path_sync_ui_srcs}
)

target_include_directories(path_sync_ui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/libs/path_sync_core/include
    ${CMAKE_SOURCE_DIR}/app/src
)

find_package(Qt6 REQUIRED COMPONENTS Widgets)

target_link_libraries(path_sync_ui PUBLIC
    Qt6::Widgets
    path_sync_core
    yaml-cpp
)
