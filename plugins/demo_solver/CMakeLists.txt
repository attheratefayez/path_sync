cmake_minimum_required(VERSION 3.20)
project(demo_solver_plugin)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Point this to the project root so <path_sync_core/...> includes resolve
set(PATH_SYNC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")

add_library(demo_solver SHARED demo_solver.cpp)
target_include_directories(demo_solver PRIVATE "${PATH_SYNC_ROOT}/libs/path_sync_core/include")
target_compile_definitions(demo_solver PRIVATE PATH_SYNC_BUILD_AS_PLUGIN)

# The -rpath trick lets dlopen find libpath_sync_core.so at runtime
set_target_properties(demo_solver PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../"
    BUILD_RPATH "${PATH_SYNC_ROOT}/build/libs/path_sync_core"
    INSTALL_RPATH "${PATH_SYNC_ROOT}/build/libs/path_sync_core"
)
