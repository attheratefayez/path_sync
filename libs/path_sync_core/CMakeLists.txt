file(GLOB path_sync_core_srcs "./src/*.cpp")

add_library(path_sync_core SHARED
    ${path_sync_core_srcs}
)

target_include_directories(path_sync_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_definitions(path_sync_core PUBLIC PROJECT_ROOT="${CMAKE_SOURCE_DIR}")


