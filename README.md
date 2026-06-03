# PathSync

A C++20 pathfinding visualization tool for single-agent and multi-agent pathfinding algorithms on grid-based maps. Supports multiple solvers with performance metrics and real-time visualization via SFML.

## Algorithms

### Single-Agent Solvers
- **A\*** - Classic A\* with Manhattan distance heuristic
- **BFS** - Breadth-First Search (unweighted grids)

### Multi-Agent Solvers
- **Joint-State A\*** - Multi-agent pathfinding in joint state space (WIP)

## Dependencies

| Dependency | Version | Install |
|---|---|---|
| SFML | 3.0.0+ | Pre-built in `libs/path_sync_ui/lib/SFML/` |
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
│   │   │   ├── solvers/          # A*, BFS, Joint-State A*
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
| `Shift+H` | Help overlay |

## Map Format

Maps use the standard [Moving AI](https://movingai.com/benchmarks/) grid format (`.map` + `.map.scen`).

## License

MIT
