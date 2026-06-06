# PathSync

A C++20 pathfinding visualization tool for single-agent and multi-agent pathfinding algorithms on grid-based maps. Supports multiple solvers with performance metrics and real-time visualization via Qt6.

## Algorithms

### Single-Agent Solvers
- **A\*** (optimal) — Classic A\* with Manhattan distance heuristic (4-dir)
- **BFS** (optimal) — Breadth-First Search for unweighted grids (4-dir)
- **JPS** (optimal) — Jump Point Search with 8-directional movement and Chebyshev heuristic
- **D\* Lite** (optimal) — Incremental replanning from goal-to-start (8-dir)
- **EPEA\*** (optimal) — Enhanced Partial Expansion A\* with delayed successor generation (4-dir)
- **Theta\*** (suboptimal) — Any-angle pathfinding with line-of-sight shortcutting (8-dir)
- **HPA\*** (suboptimal) — Hierarchical Pathfinding A\* using cluster decomposition (4-dir)

### Multi-Agent Solvers
- **Joint-State A\*** (optimal) — Multi-agent pathfinding in joint state space (WIP)
- **M\*** (suboptimal) — Subdimensional expansion with independent planning + conflict resolution (4-dir)
- **CBS** (optimal) — Conflict-Based Search with Constraint Tree, vertex/edge constraints (4-dir)
- **ICBS** (optimal) — Improved CBS with conflict classification (cardinal/semi/non-cardinal) and bypass
- **CBSH** (optimal) — CBS with Conflict Graph heuristic (minimum vertex cover via brute force)

## Dependencies

| Dependency | Version | Install |
|---|---|---|
| Qt6 | 6.5+ | `sudo apt install qt6-base-dev` |
| yaml-cpp | - | `sudo apt install libyaml-cpp-dev` |
| Doxygen | - | `sudo apt install doxygen` (docs only) |
| GoogleTest | - | Fetched automatically by CMake (tests only) |

## Build

```bash
cmake -B build -S .
cmake --build build
```

All solver plugins are built automatically alongside the main binary and placed in `plugins/`.

Run:
```bash
./build/path_sync
```

Run tests:
```bash
./build/path_sync_test
```

## Architecture

The application is split into three layers:

```
┌──────────────────────────────────────────────┐
│  app/   (PathSyncApp, main)                   │
│  Orchestrates core + UI, connects signals     │
├──────────────────────────────────────────────┤
│  libs/path_sync_ui/   (Qt6 visualization)     │
│  Grid widget, toolbar, sidebar, scene mgmt    │
├──────────────────────────────────────────────┤
│  libs/path_sync_core/   (algorithms, data)    │
│  Solvers, MapManager, PathFinder,             │
│  PluginLoader, PerformanceMetrics             │
└──────────────────────────────────────────────┘
```

### Data flow

1. **Map loading** — `MapManager` parses `.map` + `.map.scen` files into `MapData` (grid of `CellType`) and scenes.
2. **Solving** — User clicks **Solve** → `PathFinder::find_path()` → selects the current solver (built-in or plugin) → runs the algorithm → returns a path or paths.
3. **Plugin discovery** — On construction, `PathFinder` calls `PluginLoader::load_plugins("plugins/")` which `dlopen`s each `.so`, queries its `extern "C"` symbols, and adds it to the solver list. **All solvers** (both built-in and external) are compiled as standalone `.so` plugins in `plugins/` — there is zero static registration. Each solver appears in the same UI dropdown.

```
Map files → MapData  →  Grid (visual)
                  ↘
     PathFinder ──→ Solver (ISolver / IMASolver)
       │                 built-in or plugin .so
       │                 │
       ▼                 ▼
  PerformanceMetrics   Path result → Grid overlay
       │
       ▼
    Sidebar (last 5 runs)
```

## Project Structure

```
├── app/                          # Application layer
│   └── src/
│       ├── PathSyncApp.hpp/.cpp  # Core app logic
│       └── main.cpp              # Standalone CLI entry
├── libs/
│   ├── path_sync_core/           # Core library
│   │   ├── include/path_sync_core/
│   │   │   ├── map_loader/       # Map & scene parsing
│   │   │   ├── solvers/          # A*, BFS, JPS, Theta*, HPA*, D* Lite, EPEA*, Joint-State A*, M*, CBS
│   │   │   ├── logger.hpp              # Singleton logger
│   │   │   ├── path_sync_types.hpp
│   │   │   ├── plugin_loader.hpp       # Dynamic plugin system
│   │   │   └── performance_mat.hpp     # Performance metrics struct
│   │   └── src/
│   └── path_sync_ui/             # Qt6 visualization
│       ├── include/path_sync_ui/
│       └── src/
├── maps/                         # Grid map files
├── config/                       # YAML configuration
├── log/                          # Solver performance logs
├── plugins/                      # Solver .so plugins (built-in + external)
│   ├── CMakeLists.txt            # Builds all built-in solvers as plugins
│   ├── demo_solver/              # Example plugin template
│   ├── libastar_solver.so
│   ├── libbfs_solver.so
│   ├── libjps_solver.so
│   ├── libtheta_star_solver.so
│   ├── libhpa_solver.so
│   ├── libdstar_lite_solver.so
│   ├── libepea_solver.so
│   ├── libastar_joint_state.so
│   ├── libmstar_solver.so
│   ├── libcbs_solver.so           # plain CBS
│   ├── libicbs_solver.so          # ICBS (conflict classification + bypass)
│   └── libcbsh_solver.so          # CBS + ICBS + CBSH (CG heuristic)
└── test/                         # GoogleTest unit tests
```

## Usage

### Controls

**Toolbar buttons:**
| Widget | Action |
|---|---|
| `Solve` | Solve current scene |
| `Cancel` | Cancel a running solver |
| `Clear` | Clear path overlay |
| `Reset` | Reset grid to original map |
| `Current Scene` spin box | Jump to / step through scenes |
| `Map` spin box | Jump to / cycle maps |
| `Agent` | Cycle agent count (1–10) |
| `Solver` dropdown | Select solver directly (shows optimal/suboptimal) |

**Keyboard shortcuts:**
None. All operations are available via the toolbar.

**Mouse controls:**
| Action | Input |
|---|---|
| Toggle wall cell | Left-click |
| Draw walls | Left-click drag |

### Performance Sidebar
A permanent sidebar (320 px, right of the viewport) displays the last 5 solver runs with timing, search effort, and path quality metrics. The buffer resets automatically when the map or scene changes.

## Map Format

Maps use the standard [Moving AI](https://movingai.com/benchmarks/) grid format (`.map` + `.map.scen`).

## Extending

PathSync loads solver algorithms dynamically via a plugin system. Add your own single-agent or multi-agent solver as a `.so` shared library — no core source changes needed.

See **[HOW_TO_ADD_SOLVER.md](HOW_TO_ADD_SOLVER.md)** for a step-by-step guide with code examples.

The [`plugins/demo_solver/`](plugins/demo_solver/) directory contains a complete working plugin (Greedy Best-First Search) that you can use as a template.

## License

GNU General Public License v3.0
