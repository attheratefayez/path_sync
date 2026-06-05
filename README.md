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

Run:
```bash
./build/path_sync
```

Run tests:
```bash
./build/path_sync_test
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
│   │   │   ├── solvers/          # A*, BFS, JPS, Theta*, HPA*, D* Lite, EPEA*, Joint-State A*, M*
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
├── plugins/                      # External solver .so plugins
│   └── demo_solver/              # Example plugin template
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

MIT
