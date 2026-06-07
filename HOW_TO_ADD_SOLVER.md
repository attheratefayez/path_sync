# How to Add Your Own Solver

PathSync loads solvers dynamically via a plugin system. You can add your own algorithm as a `.so` shared library without modifying the core source code.

## Step-by-Step

### 1. Pick your interface

| Interface | Solver type | Method signature |
|---|---|---|
| `ISolver` | Single-agent | `solve(MapData, start, goal, PerformanceMetrics) -> map<Coordinate,Coordinate>` |
| `IMASolver` | Multi-agent | `solve(MapData, starts, goals, PerformanceMetrics) -> optional<vector<vector<Coordinate>>>` |
| `IMOSolver` | Multi-objective | `solve(MapData, CostMap*, start, goal, n_obj, PerformanceMetrics, MOMetrics) -> optional<vector<MOSolution>>` |

SA and MA interfaces are defined in [`solver_interface.hpp`](libs/path_sync_core/include/path_sync_core/solver_interface.hpp).
The MO interface is defined in [`imo_solver.hpp`](libs/path_sync_core/include/path_sync_core/solvers/imo_solver.hpp).

### 2. Create your solver class

```cpp
// my_solver.cpp
#include <path_sync_core/solver_interface.hpp>

class MySolver : public ISolver {
public:
    std::string_view get_solver_name() const override {
        return "My Custom Solver";
    }

    bool is_optimal() const override { return false; }

    std::map<Coordinate, Coordinate> solve(
        const path_sync::MapData &map_data,
        Coordinate start, Coordinate goal,
        path_sync::PerformanceMetrics &performance_met) override
    {
        // Your algorithm here...
        // Return came_from map (empty if no path found)
    }
};
```

> **Tip**: See [`plugins/demo_solver/demo_solver.cpp`](plugins/demo_solver/demo_solver.cpp) for a complete working example (Greedy Best-First Search).

### 3. Populate performance metrics (required for data collection)

PathSync records solver performance for research. Your solver **must** populate the `PerformanceMetrics` struct received via the `solve()` parameter. Fields are split into two categories:

#### Fields your solver must set (inside `solve()`)

| Field | Type | When to set |
|---|---|---|
| `success` | `bool` | `true` if a path was found |
| `runtime` | `std::chrono::microseconds` | Elapsed wall-clock time |
| `num_of_nodes_explored` | `std::size_t` | Total successor states generated |
| `num_of_nodes_expanded` | `std::size_t` | Total states popped from the open set |
| `num_of_nodes_reopened` | `std::size_t` | States reached again with a worse g-score |
| `peak_open_size` | `std::size_t` | Maximum size of the open priority queue |
| `path_length` | `std::size_t` | Length of the final path (in steps) |

#### Fields populated externally (do NOT set in solver)

| Field | Set by |
|---|---|
| `solver_name` | PathFinder |
| `map_name` | PathFinder |
| `scene_id` | PathFinder |
| `num_agents` | PathFinder |
| `timestamp` | PathFinder |
| `optimal_path_length` | External comparison (benchmarking scripts) |
| `sum_of_costs` | PathFinder (for multi-agent) |
| `makespan` | PathFinder (for multi-agent) |
| `cancel_flag` | PathFinder (non-owning pointer to atomic flag) |

#### Complete example

```cpp
auto start_time = std::chrono::high_resolution_clock::now();
// ... run algorithm ...
auto end_time = std::chrono::high_resolution_clock::now();

performance_met.success = found;
performance_met.runtime = std::chrono::duration_cast<
    std::chrono::microseconds>(end_time - start_time);
performance_met.path_length = final_path.size();
performance_met.num_of_nodes_explored = explored_count;
performance_met.num_of_nodes_expanded = expanded_count;
performance_met.num_of_nodes_reopened = reopened_count;
performance_met.peak_open_size = peak_open;

if (!found)
    came_from.clear();  // signal failure
return came_from;
```

The demo plugin at [`plugins/demo_solver/demo_solver.cpp`](plugins/demo_solver/demo_solver.cpp) shows this in full context.

#### Multi-agent extra fields

For `IMASolver`, also set `sum_of_costs` and `makespan` on the metrics struct if a solution was found:

```cpp
performance_met.sum_of_costs = soc;   // sum of (agent_i_path_length - 1)
performance_met.makespan = makespan;  // max(agent_i_path_length - 1)
```

These are also computed externally by PathFinder, but setting them in the solver is good practice for when the solver is used as a standalone library.

### 4. Add the plugin entry points (required)

These `extern "C"` functions are the contract between PathSync and your plugin:

```cpp
extern "C" {

const char *plugin_name()          { return "My_Solver"; }   // Display name
bool     plugin_is_optimal()       { return false; }          // true if provably optimal
bool     plugin_is_multi_agent()   { return false; }          // true for MAPF solvers
void    *plugin_create()           { return new MySolver(); } // Factory
void     plugin_destroy(void *p)   { delete static_cast<MySolver*>(p); }

}
```

### 5. Create a CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_solver_plugin)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(PATH_SYNC_ROOT "/path/to/path_sync")

add_library(my_solver SHARED my_solver.cpp)
target_include_directories(my_solver PRIVATE
    "${PATH_SYNC_ROOT}/libs/path_sync_core/include")
target_compile_definitions(my_solver PRIVATE PATH_SYNC_BUILD_AS_PLUGIN)

set_target_properties(my_solver PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${PATH_SYNC_ROOT}/plugins/"
    BUILD_RPATH  "${PATH_SYNC_ROOT}/build/libs/path_sync_core"
    INSTALL_RPATH "${PATH_SYNC_ROOT}/build/libs/path_sync_core")
```

### 6. Build

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

This produces `libmy_solver.so` in the `plugins/` directory of the PathSync project.

### 7. Verify

Start PathSync. Your solver should appear in the **Solver** dropdown menu under the corresponding section (single-agent or multi-agent).

## What happens at startup?

1. `PathFinder` registers all built-in solvers programmatically.
2. It calls `plugin_loader_.load_plugins("plugins/")` which scans for `.so` files.
3. For each `.so`, it `dlopen`s the library, looks up the `extern "C"` symbols, and creates an instance.
4. Solvers are merged into the same solver lists — your plugin gets the same UI treatment as built-in solvers.

## Plugin ABI Contract

Your `.so` must export these symbols:

| Symbol | Type | Mandatory | Purpose |
|---|---|---|---|
| `plugin_name` | `const char* ()` | yes | Display name in UI |
| `plugin_is_optimal` | `bool ()` | yes | Optimality flag |
| `plugin_is_multi_agent` | `bool ()` | yes | Multi-agent flag (false for SA/MO) |
| `plugin_is_mo` | `bool ()` | for MO | Multi-objective flag — set to `true` for MO solvers |
| `plugin_create` | `void* ()` | yes | Factory — returns new solver instance |
| `plugin_destroy` | `void* (void*)` | yes | Destructor — cast and delete the instance |

## Multi-agent solver

For MAPF, inherit from `IMASolver` instead and set `plugin_is_multi_agent()` to return `true`:

```cpp
class MyMASolver : public IMASolver {
    std::string_view get_solver_name() const override {
        return "My MA Solver";
    }

    std::optional<std::vector<std::vector<Coordinate>>> solve(
        const path_sync::MapData &map_data,
        std::vector<Coordinate> starts,
        std::vector<Coordinate> goals,
        path_sync::PerformanceMetrics &performance_met) override
    {
        // Return nullopt if no solution
        // Populate metrics (see step 3) plus:
        //   performance_met.sum_of_costs
        //   performance_met.makespan
    }
};

extern "C" {
    const char *plugin_name()          { return "My_MA_Solver"; }
    bool plugin_is_optimal()           { return true; }
    bool plugin_is_multi_agent()       { return true; }
    void *plugin_create()              { return new MyMASolver(); }
    void  plugin_destroy(void *p)      { delete static_cast<MyMASolver*>(p); }
}
```

## Multi-objective solver

For MO (multi-objective) pathfinding, inherit from `IMOSolver` (defined in
[`imo_solver.hpp`](libs/path_sync_core/include/path_sync_core/solvers/imo_solver.hpp))
and export `plugin_is_mo()` returning `true`:

```cpp
#include <path_sync_core/solvers/imo_solver.hpp>
#include <path_sync_core/mo_types.hpp>

class MyMOSolver : public IMOSolver {
    std::string_view get_solver_name() const override {
        return "My MO Solver";
    }

    bool is_optimal() const override { return false; }

    int get_num_objectives() const override { return 5; }
    bool needs_weights() const override { return false; }
    void set_weights(const std::vector<float> &) override {}

    std::optional<std::vector<MOSolution>> solve(
        const path_sync::MapData &map_data,
        const path_sync::CostMap *cost_map,   // nullable — nullptr = uniform costs
        path_sync::Coordinate start,
        path_sync::Coordinate goal,
        int num_objectives,
        path_sync::PerformanceMetrics &perf,
        path_sync::MOMetrics &mo_met) override
    {
        // cost_map is null when no .cost file exists; the solver should
        // fall back to uniform costs (all objectives = 1.0 per step).
        // For custom maps, PathSyncApp generates a CostMap on-the-fly,
        // so cost_map is usually non-null in practice.
        //
        // Return a vector of non-dominated MOSolution objects (Pareto front).
        // Populate perf and mo_met (see below).
    }
};

extern "C" {
    const char *plugin_name()        { return "My_MO_Solver"; }
    bool plugin_is_optimal()         { return false; }
    bool plugin_is_multi_agent()     { return false; }
    bool plugin_is_mo()              { return true; }   // ← required!
    void *plugin_create()            { return new MyMOSolver(); }
    void  plugin_destroy(void *p)    { delete static_cast<MyMOSolver*>(p); }
}
```

### MOMetrics fields to populate

| Field | Type | When to set |
|---|---|---|
| `front_size` | `int` | Size of the returned Pareto front |
| `front` | `std::vector<MOSolution>` | The full Pareto front |
| `ref_point` | `std::vector<float>` | Reference point for hypervolume (one per objective) |
| `hypervolume` | `double` | Hypervolume indicator of the front |

See [`mo_types.hpp`](libs/path_sync_core/include/path_sync_core/mo_types.hpp)
for `MOSolution` (path + cost vector + crowding distance) and
[`imo_solver.hpp`](libs/path_sync_core/include/path_sync_core/solvers/imo_solver.hpp)
for the full interface.

### Plugin ABI contract summary

| Symbol | SA | MA | MO |
|---|---|---|---|
| `plugin_is_multi_agent` | `false` | **`true`** | `false` |
| `plugin_is_mo` | _omit_ | _omit_ | **`true`** |

## Loading errors

If a plugin fails to load, the error message is logged to stderr. Check the console output for details (`dlopen` errors, missing symbols, constructor exceptions).
