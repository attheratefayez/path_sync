# adding this to stop cmake bitching about DOWNLOAD_EXTRACT_TIMESTAMP 
# when using URL in FetchContent
cmake_policy(SET CMP0135 NEW) 

include(FetchContent)
FetchContent_Declare(
    google_test
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(google_test)

set(test_src_files
    ./solvers/astar_joint_state_utils.cpp
    ./main_test.cpp
)


enable_testing()
add_executable(path_sync_test
    ${test_src_files}
)

target_link_directories(path_sync_test
    PUBLIC ${CMAKE_SOURCE_DIR}/lib/SFML/
)

target_link_libraries(path_sync_test
    GTest::gtest_main
    sfml-graphics
    sfml-window
    sfml-system
    yaml-cpp
    path_sync_src
)

include(GoogleTest)
gtest_discover_tests(path_sync_test)
