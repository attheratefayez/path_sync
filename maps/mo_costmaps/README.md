# Multi-Objective Cost Maps

Generated from MovingAI `.map` files by `scripts/generate_cost_map_layers.py`
or computed on-the-fly for custom maps inside `PathSyncApp::generate_cost_map_from_map_data()`.

Each `.cost` file (or generated in-memory `CostMap`) provides 5 synthetic cost objectives
for multi-objective pathfinding. The C++ on-the-fly generator uses a BFS distance transform
from wall cells to compute obstacle proximity — no random noise, fully deterministic.

## Format (binary)

```
Offset  Size  Field
──────────────────────────────────────
 0       4     height       (int32, little-endian)
 4       4     width        (int32, little-endian)
 8       4     objectives   (int32, little-endian)
12       N     costs        (float32, row-major)
```

N = `height × width × objectives × sizeof(float)`

Cost array layout: `costs[obj][y][x]`, i.e. all cells for objective 0,
then all cells for objective 1, etc.

Blocked cells (walls) have **all objectives = -1.0**.

## Objectives

| Index | Name       | Description |
|-------|------------|-------------|
| 0     | Distance   | 1.0 base, 1.5× on `T` terrain |
| 1     | Risk       | Obstacle proximity `1 + 9/(1+dist)`, ×2.25 if ≤2 free neighbors (bottleneck/dead-end penalty) |
| 2     | Energy     | Terrain-based (`T` = 3×), scaled by obstacle proximity |
| 3     | Visibility | `max(1, 10 - dist_to_wall)` — open spaces are cheap |
| 4     | Terrain    | `T` cells cost 5× + wall-edge bonus within 3 cells |

The **Python script** additionally applies corner penalties and narrower bottleneck factors.
The **C++ generator** (`PathSyncApp::generate_cost_map_from_map_data`) is a
lighter version used when no `.cost` file exists (e.g. custom maps).

## Loading in C++

```cpp
#include "path_sync_core/map_loader/cost_map.hpp"

path_sync::CostMap cm;
if (cm.load("maps/mo_costmaps/arena2.cost")) {
    float risk_at_10_20 = cm.at(/*obj=*/1, /*x=*/10, /*y=*/20);
    bool blocked = cm.is_blocked(50, 60);
}
```

`CostMap` is declared in `libs/path_sync_core/include/path_sync_core/map_loader/cost_map.hpp`
and implemented in `libs/path_sync_core/src/cost_map.cpp`. It is auto-linked via `path_sync_core`.

## Regenerating

From the project root:

```bash
python3 scripts/generate_cost_map_layers.py
```

Requires `numpy`. All objectives are computed from map structure and are fully
deterministic (no random seed parameter needed).

Cost maps are automatically generated at runtime for any map without a `.cost` file
via `PathSyncApp::load_cost_map()` → `generate_cost_map_from_map_data()`.
