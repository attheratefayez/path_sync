cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)

project(path_sync
    DESCRIPTION "Visualizing and Analyzing Path Finding Algorithm."
    VERSION 0.0.1
    LANGUAGES CXX
)

add_subdirectory(src)
add_subdirectory(lib)
# add_subdirectory(test)

add_executable(${PROJECT_NAME}
    main.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/SFML/
)

target_link_libraries(${PROJECT_NAME}
    sfml-graphics
    sfml-system
    sfml-window
    yaml-cpp
    path_sync_src
)

