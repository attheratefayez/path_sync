file(GLOB PATH_SYNC_SOURCE_FILES "./path_sync/*.cpp")

add_library(path_sync_src
    SHARED ${PATH_SYNC_SOURCE_FILES}
)

# to make PROJECT_ROOT available system wide
target_compile_definitions(path_sync_src
    PUBLIC PROJECT_ROOT="${CMAKE_SOURCE_DIR}"
)

target_include_directories(path_sync_src
    PUBLIC ${CMAKE_SOURCE_DIR}/include
)
