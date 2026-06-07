import os
import struct
import glob

import numpy as np
from scipy.ndimage import gaussian_filter, distance_transform_edt





def load_movingai_map(path):
    with open(path, "r") as f:
        lines = [x.rstrip() for x in f]

    idx = lines.index("map")
    grid = lines[idx + 1:]

    h = len(grid)
    w = len(grid[0])

    cell_type = np.zeros((h, w), dtype=np.int32)
    traversable = np.zeros((h, w), dtype=np.bool_)

    for y in range(h):
        for x in range(w):
            ch = grid[y][x]
            if ch == '.':
                traversable[y, x] = True
                cell_type[y, x] = 0
            elif ch == 'T':
                traversable[y, x] = True
                cell_type[y, x] = 1
            elif ch == '@':
                traversable[y, x] = False
                cell_type[y, x] = -1

    return traversable, cell_type


def obstacle_distance(traversable):
    return distance_transform_edt(traversable)


def neighbor_obstacle_count(traversable):
    obs = (~traversable).astype(np.float32)
    count = np.zeros_like(obs)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            shifted = np.roll(obs, shift=(-dy, -dx), axis=(0, 1))
            count += shifted
    return count  # 0-8


def cardinal_obstacle_count(traversable):
    obs = (~traversable).astype(np.float32)
    count = np.zeros_like(obs)
    for dy, dx in [(-1,0),(1,0),(0,-1),(0,1)]:
        shifted = np.roll(obs, shift=(-dy, -dx), axis=(0, 1))
        count += shifted
    return count  # 0-4


def corner_penalty(traversable):
    obs = (~traversable).astype(np.float32)
    corners = np.zeros_like(obs)
    for dy in (-1, 1):
        for dx in (-1, 1):
            shifted = np.roll(obs, shift=(-dy, -dx), axis=(0, 1))
            corners += shifted
    return corners / 4.0  # 0-1


def corridor_bottleneck(traversable):
    """Bottleneck score: a cell in a narrow corridor flanked by walls."""
    card = cardinal_obstacle_count(traversable)
    narrow = neighbor_obstacle_count(traversable)

    opp_walls = (card >= 2).astype(np.float32)

    ring5 = np.zeros_like(narrow)
    for dy in (-2, -1, 0, 1, 2):
        for dx in (-2, -1, 0, 1, 2):
            d = abs(dx) + abs(dy)
            if d == 0 or d > 2:
                continue
            shifted = np.roll((~traversable).astype(np.float32), shift=(-dy, -dx), axis=(0, 1))
            ring5 += shifted

    pinch = opp_walls * np.clip(ring5 / 16.0, 0, 1)
    return pinch  # 0-1


def dead_end_penalty(traversable):
    """Cells that lead to a dead end (3+ obstacles in cardinal dirs)."""
    card = cardinal_obstacle_count(traversable)
    return (card >= 3).astype(np.float32)


def visibility_cost(traversable):
    obs_dist = obstacle_distance(traversable)
    safe = np.percentile(obs_dist[obs_dist > 0.5], 95)
    inv = safe - np.clip(obs_dist, 0, safe)
    return np.clip(inv / safe, 0, 1)


def terrain_energy_cost(cell_type):
    energy = np.ones_like(cell_type, dtype=np.float32)
    energy[cell_type == 1] = 3.0
    return energy


def risk_cost(obs_dist, neighbor_obs, corner, bottleneck, dead_end):
    prox = 1.0 + 9.0 / (1.0 + obs_dist)

    wedged = np.where(neighbor_obs >= 5,
                      6.0 * ((neighbor_obs - 4) / 4.0) ** 2.0,
                      np.zeros_like(neighbor_obs))

    pinch = bottleneck * 6.0

    trapped = dead_end * 3.0

    blind = corner * 1.0

    cost = prox + wedged + pinch + trapped + blind
    return np.clip(cost, 1.0, 10.0)


def energy_cost(obs_dist, cell_type):
    base = terrain_energy_cost(cell_type)
    terrain_contrib = base
    dist_contrib = 1.0 + 0.5 * np.exp(-obs_dist * 0.5)
    return np.clip(terrain_contrib * dist_contrib, 1.0, 10.0)


def visibility_cost_layer(vis):
    return np.clip(1.0 + 9.0 * vis, 1.0, 10.0)


def terrain_cost_layer(cell_type, obs_dist):
    base = np.ones_like(cell_type, dtype=np.float32)
    base[cell_type == 1] = 5.0
    edge_bonus = 1.0 + 2.0 * np.exp(-obs_dist * 0.4)
    cost = base * edge_bonus
    return np.clip(cost, 1.0, 10.0)


def distance_cost_layer(cell_type):
    cost = np.ones_like(cell_type, dtype=np.float32)
    cost[cell_type == 1] = 1.5
    return cost


def generate_cost_layers(traversable, cell_type, objectives=5):
    h, w = traversable.shape

    obs_dist = obstacle_distance(traversable)
    neighbor_obs = neighbor_obstacle_count(traversable)
    corner = corner_penalty(traversable)
    bottleneck = corridor_bottleneck(traversable)
    dead_end = dead_end_penalty(traversable)
    vis = visibility_cost(traversable)

    layers = []

    if objectives >= 1:
        layers.append(distance_cost_layer(cell_type).astype(np.float32))

    if objectives >= 2:
        layers.append(risk_cost(obs_dist, neighbor_obs, corner, bottleneck, dead_end).astype(np.float32))

    if objectives >= 3:
        layers.append(energy_cost(obs_dist, cell_type).astype(np.float32))

    if objectives >= 4:
        layers.append(visibility_cost_layer(vis).astype(np.float32))

    if objectives >= 5:
        layers.append(terrain_cost_layer(cell_type, obs_dist).astype(np.float32))

    layers = np.stack(layers)

    layers[:, ~traversable] = -1.0

    return layers


def save_binary_cost_map(filename, cost_layers):
    objectives, height, width = cost_layers.shape

    with open(filename, "wb") as f:
        f.write(struct.pack("<iii", height, width, objectives))
        cost_layers.astype(np.float32).tofile(f)


def process_maps(maps_root, output_root, objectives=5):
    os.makedirs(output_root, exist_ok=True)

    map_files = sorted(glob.glob(os.path.join(maps_root, "**", "*.map"), recursive=True))

    print(f"Found {len(map_files)} maps")

    for idx, map_path in enumerate(map_files):
        traversable, cell_type = load_movingai_map(map_path)

        layers = generate_cost_layers(traversable, cell_type, objectives=objectives)

        rel = os.path.relpath(map_path, maps_root)
        out_file = os.path.splitext(rel)[0] + ".cost"
        out_file = os.path.join(output_root, out_file)
        os.makedirs(os.path.dirname(out_file), exist_ok=True)

        save_binary_cost_map(out_file, layers)

        print(f"[{idx+1}/{len(map_files)}] {out_file}")


if __name__ == "__main__":
    import sys

    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)

    maps_root = os.path.join(project_root, "maps")
    output_root = os.path.join(project_root, "maps", "mo_costmaps")

    objectives = int(sys.argv[1]) if len(sys.argv) > 1 else 5

    process_maps(maps_root=maps_root, output_root=output_root, objectives=objectives)
