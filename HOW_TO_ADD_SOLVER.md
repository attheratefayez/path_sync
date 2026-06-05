# How to Add Your Own Solver

PathSync loads solvers dynamically via a plugin system. You can add your own algorithm as a `.so` shared library without modifying the core source code.

## Step-by-Step

### 1. Pick your interface

| Interface | Solver type | Method signature |
|---|---|---|
| `ISolver` | Single-agent | `solve(MapData, start, goal, PerformanceMetrics) -> map<Coordinate,Coordinate>` |
| `IMASolver` | Multi-agent | `solve(MapData, starts, goals, PerformanceMetrics) -> optional<vector<vector<Coordinate>>>` |

Both inherit from abstract base classes in [`solver_interface.hpp`](libs/path_sync_core/include/path_sync_core/solver_interface.hpp).

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
        // Populate performance_met with:
        //   .success, .runtime, .nodes_explored, .nodes_expanded,
        //   .nodes_reopened, .peak_open_size
        // Return came_from map (empty if no path found)
    }
};
```

> **Tip**: See [`plugins/demo_solver/demo_solver.cpp`](plugins/demo_solver/demo_solver.cpp) for a complete working example (Greedy Best-First Search).

### 3. Add the plugin entry points (required)

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

### 4. Create a CMakeLists.txt

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

### 5. Build

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

This produces `libmy_solver.so` in the `plugins/` directory of the PathSync project.

### 6. Verify

Start PathSync. Your solver should appear in the **Solver** dropdown menu under the corresponding section (single-agent or multi-agent).

## What happens at startup?

1. `PathFinder` registers all built-in solvers programmatically.
2. It calls `plugin_loader_.load_plugins("plugins/")` which scans for `.so` files.
3. For each `.so`, it `dlopen`s the library, looks up the `extern "C"` symbols, and creates an instance.
4. Solvers are merged into the same solver lists — your plugin gets the same UI treatment as built-in solvers.

## Plugin ABI Contract

Your `.so` must export exactly these 5 symbols:

| Symbol | Type | Purpose |
|---|---|---|
| `plugin_name` | `const char* ()` | Display name in UI |
| `plugin_is_optimal` | `bool ()` | Optimality flag |
| `plugin_is_multi_agent` | `bool ()` | Multi-agent flag |
| `plugin_create` | `void* ()` | Factory — returns new solver instance |
| `plugin_destroy` | `void* (void*)` | Destructor — cast and delete the instance |

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
        // Populate performance_met.sum_of_costs and .makespan if solved
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

## Loading errors

If a plugin fails to load, the error message is logged to stderr. Check the console output for details (`dlopen` errors, missing symbols, constructor exceptions).
