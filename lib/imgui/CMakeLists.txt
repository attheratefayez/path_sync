cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS True)

project(imgui
    DESCRIPTION "ImGUI library."
    LANGUAGES CXX
)

file(GLOB SOURCE_FILES "*.cpp")

add_library(${PROJECT_NAME} SHARED
    ${SOURCE_FILES}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_SOURCE_DIR}/include/
    PUBLIC ${CMAKE_SOURCE_DIR}/include/imgui
    PUBLIC ${CMAKE_SOURCE_DIR}/include/imgui/backends
)

target_link_libraries(${PROJECT_NAME}
    GL
)

message("[CREATED SHARED LIBRARY: imgui]")
