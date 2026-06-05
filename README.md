# PathSync

A C++20 pathfinding visualization tool for single-agent and multi-agent pathfinding algorithms on grid-based maps. Supports multiple solvers with performance metrics and real-time visualization via SFML.

## Algorithms

### Single-Agent Solvers
- **A\*** - Classic A\* with Manhattan distance heuristic (4-dir)
- **BFS** - Breadth-First Search for unweighted grids (4-dir)
- **JPS** - Jump Point Search with 8-directional movement and Chebyshev heuristic
- **Theta\*** - Any-angle pathfinding with line-of-sight shortcutting (8-dir)
- **HPA\*** - Hierarchical Pathfinding A\* using cluster decomposition (4-dir)
- **D\* Lite** - Incremental replanning from goal-to-start (8-dir)
- **EPEA\*** - Enhanced Partial Expansion A\* with delayed successor generation (4-dir)

### Multi-Agent Solvers
- **Joint-State A\*** - Multi-agent pathfinding in joint state space (WIP)
- **M\*** - Subdimensional expansion with independent planning + conflict resolution (4-dir)

## Dependencies

| Dependency | Version | Install |
|---|---|---|
| SFML | 3.0.0+ | Pre-built in `libs/path_sync_ui/lib/SFML/` |
| yaml-cpp | - | `sudo apt install libyaml-cpp-dev` |
| Doxygen | - | `sudo apt install doxygen` (docs only) |
| GoogleTest | - | Fetched automatically by CMake (tests only) |

## Dependencies Installation

```bash
sudo apt install cmake libyaml-cpp-dev
```

## Build

```bash
cmake -B build -S .
cmake --build build -j$(nproc)
```

Run:
```bash
./build/path_sync
```

Run tests:
```bash
./build/path_sync_test
```

### Docker (headless)

```bash
docker compose up -d
docker exec cpp-dev cmake -B /workspace/build -S /workspace -DCMAKE_BUILD_TYPE=Release
docker exec cpp-dev cmake --build /workspace/build -j$(nproc)
docker exec cpp-dev /workspace/build/test/path_sync_test
```

> **Note:** The GUI requires an X11 server. Use `xvfb-run` inside the container for headless testing:
> `docker exec cpp-dev xvfb-run /workspace/build/path_sync`

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
│   └── path_sync_ui/             # SFML visualization
│       ├── include/path_sync_ui/
│       └── src/
├── maps/                         # Grid map files
├── config/                       # YAML configuration
├── log/                          # Solver performance logs
└── test/                         # GoogleTest unit tests
```

## Usage

### Controls (SFML window)

| Key | Action |
|---|---|
| `Space` | Solve current scene |
| `C` | Cycle solvers |
| `Shift+M` | Next map |
| `Shift+P` | Clear paths |
| `Shift+R` | Reset grid |
| `]` | Next scene |
| `[` | Previous scene |
| `A` | Cycle agent count (1–10) |
| `Shift+H` | Help overlay |

## Map Format

Maps use the standard [Moving AI](https://movingai.com/benchmarks/) grid format (`.map` + `.map.scen`).

## License

MIT
