cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)

project(path_sync
    DESCRIPTION "Visualizing and Analyzing Path Finding Algorithm."
    VERSION 0.0.1
    LANGUAGES CXX
)

file(GLOB PATH_SYNC_SOURCE_FILES "./src/path_sync/*.cpp")

add_subdirectory(lib)

add_executable(${PROJECT_NAME}
    main.cpp
    ${PATH_SYNC_SOURCE_FILES}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# to make PROJECT_ROOT available system wide
target_compile_definitions(${PROJECT_NAME}
    PRIVATE PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/SFML/

)

target_link_libraries(${PROJECT_NAME}
    sfml-graphics
    sfml-window
    sfml-system
    yaml-cpp
)

