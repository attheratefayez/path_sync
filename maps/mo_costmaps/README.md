# Multi-Objective Cost Maps

Generated from MovingAI `.map` files by `scripts/generate_cost_map_layers.py`.
Each `.cost` file provides 5 synthetic cost objectives for multi-objective pathfinding.

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

| Index | Name        | Description                                            |
|-------|-------------|--------------------------------------------------------|
| 0     | Distance    | Uniform 1.0 per step (path length)                     |
| 1     | Risk        | 1.0–10.0, spatially correlated noise (danger zones)    |
| 2     | Energy      | 1.0–10.0, mix of risk + independent noise              |
| 3     | Visibility  | 1.0–10.0, anti-correlated with risk (high risk = low vis) |
| 4     | Terrain     | 1.0–10.0, independent noise layer                      |

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
python3 scripts/generate_cost_map_layers.py [objectives] [seed]
```

- `objectives`: number of cost layers (default 5)
- `seed`: base random seed (default 12345, incremented per map)

Requires `numpy` and `scipy`.
