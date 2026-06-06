# ── Plugin builds: each solver → standalone .so for dynamic dlopen loading ──

set(CORE_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/../libs/path_sync_core/include")
set(CORE_RPATH "${CMAKE_BINARY_DIR}/libs/path_sync_core")

function(add_solver_plugin target_name)
    add_library(${target_name} SHARED ${ARGN})
    target_include_directories(${target_name} PRIVATE ${CORE_INCLUDE})
    target_compile_definitions(${target_name} PRIVATE PATH_SYNC_BUILD_AS_PLUGIN)
    target_link_libraries(${target_name} PRIVATE path_sync_core)
    set_target_properties(${target_name} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        BUILD_RPATH "${CORE_RPATH}"
        INSTALL_RPATH "${CORE_RPATH}"
    )
endfunction()

# ── Single-agent solvers ──────────────────────────────────────────────

add_solver_plugin(astar_solver
    ../libs/path_sync_core/src/astar_solver.cpp
)
add_solver_plugin(bfs_solver
    ../libs/path_sync_core/src/bfs_solver.cpp
)
add_solver_plugin(jps_solver
    ../libs/path_sync_core/src/jps_solver.cpp
)
add_solver_plugin(theta_star_solver
    ../libs/path_sync_core/src/theta_star_solver.cpp
)
add_solver_plugin(hpa_solver
    ../libs/path_sync_core/src/hpa_solver.cpp
)
add_solver_plugin(dstar_lite_solver
    ../libs/path_sync_core/src/dstar_lite_solver.cpp
)
add_solver_plugin(epea_solver
    ../libs/path_sync_core/src/epea_solver.cpp
)

# ── Multi-agent solvers ───────────────────────────────────────────────

add_solver_plugin(astar_joint_state
    ../libs/path_sync_core/src/astar_joint_state.cpp
    ../libs/path_sync_core/src/astar_joint_state_utils.cpp
)
add_solver_plugin(mstar_solver
    ../libs/path_sync_core/src/mstar_solver.cpp
)

# ── CBS family (single solver, three source files) ────────────────────

add_solver_plugin(cbs_solver
    ../libs/path_sync_core/src/cbs_solver.cpp
    ../libs/path_sync_core/src/icbs_solver.cpp
    ../libs/path_sync_core/src/cbsh_solver.cpp
)
