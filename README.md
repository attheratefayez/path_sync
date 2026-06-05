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
│   │   │   ├── logger.hpp        # Singleton logger
│   │   │   └── path_sync_types.hpp
│   │   └── src/
│   └── path_sync_ui/             # Qt6 visualization
│       ├── include/path_sync_ui/
│       └── src/
├── maps/                         # Grid map files
├── config/                       # YAML configuration
├── log/                          # Solver performance logs
└── test/                         # GoogleTest unit tests
```

## Usage

### Controls

**Toolbar buttons:**
| Button | Action |
|---|---|
| `Solve` | Solve current scene |
| `Cancel` | Cancel a running solver |
| `Clear` | Clear path overlay |
| `Reset` | Reset grid to original map |
| `< Scene` | Previous scene (hold to repeat, auto-accelerates) |
| `Scene >` | Next scene (hold to repeat, auto-accelerates) |
| `< Map` | Previous map |
| `Map >` | Next map |
| `Agent` | Cycle agent count (1–10) |
| `Solver` dropdown | Select solver directly (shows optimal/suboptimal) |

**Keyboard shortcuts:**
| Key | Action |
|---|---|
| `Space` | Solve current scene |
| `C` | Cycle solvers |
| `A` | Cycle agent count (1–10) |
| `Shift+M` | Next map |
| `Shift+P` | Clear paths |
| `Shift+R` | Reset grid |
| `]` | Next scene |
| `[` | Previous scene |
| `Shift+H` | Help overlay |
| Mouse click | Toggle wall cell |
| Mouse drag | Draw walls |

### Performance Sidebar
A permanent sidebar (320 px, right of the viewport) displays the last 5 solver runs with timing, search effort, and path quality metrics. The buffer resets automatically when the map or scene changes.

## Map Format

Maps use the standard [Moving AI](https://movingai.com/benchmarks/) grid format (`.map` + `.map.scen`).

## License

MIT
