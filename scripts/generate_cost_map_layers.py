import os
import struct
import glob

import numpy as np
from scipy.ndimage import gaussian_filter


# ---------------------------------------------------------
# MovingAI map loader
# ---------------------------------------------------------

def load_movingai_map(path):
    with open(path, "r") as f:
        lines = [x.rstrip() for x in f]

    idx = lines.index("map")

    grid = lines[idx + 1:]

    h = len(grid)
    w = len(grid[0])

    traversable = np.zeros((h, w), dtype=np.bool_)

    for y in range(h):
        for x in range(w):
            if grid[y][x] == '.':
                traversable[y, x] = True

    return traversable


# ---------------------------------------------------------
# Utilities
# ---------------------------------------------------------

def normalize(arr):
    mn = arr.min()
    mx = arr.max()

    if mx - mn < 1e-12:
        return np.zeros_like(arr)

    return (arr - mn) / (mx - mn)


def smooth_random_field(shape, rng, sigma):
    field = rng.random(shape)

    field = gaussian_filter(
        field,
        sigma=sigma,
        mode="reflect"
    )

    return normalize(field)


# ---------------------------------------------------------
# Cost generation
# ---------------------------------------------------------

def generate_cost_layers(
        traversable,
        objectives=5,
        seed=0):

    rng = np.random.default_rng(seed)

    h, w = traversable.shape

    base = smooth_random_field(
        (h, w),
        rng,
        sigma=rng.uniform(6, 20)
    )

    terrain = smooth_random_field(
        (h, w),
        rng,
        sigma=rng.uniform(6, 20)
    )

    layers = []

    #
    # Objective 1
    # Distance
    #
    distance = np.ones((h, w), dtype=np.float32)
    layers.append(distance)

    #
    # Objective 2
    # Risk
    #
    risk = 1.0 + 9.0 * base
    layers.append(risk.astype(np.float32))

    #
    # Objective 3
    # Energy
    #
    energy_noise = smooth_random_field(
        (h, w),
        rng,
        sigma=rng.uniform(6, 20)
    )

    energy = (
        0.7 * risk +
        0.3 * (1.0 + 9.0 * energy_noise)
    )

    layers.append(energy.astype(np.float32))

    #
    # Objective 4
    # Visibility
    # Anti-correlated
    #
    visibility = 1.0 + 9.0 * (1.0 - base)
    layers.append(visibility.astype(np.float32))

    #
    # Objective 5
    # Terrain
    #
    terrain_cost = 1.0 + 9.0 * terrain
    layers.append(terrain_cost.astype(np.float32))

    layers = np.stack(layers)

    #
    # Obstacles = -1
    #
    layers[:, ~traversable] = -1.0

    return layers


# ---------------------------------------------------------
# Binary writer
# ---------------------------------------------------------

def save_binary_cost_map(
        filename,
        cost_layers):

    objectives, height, width = cost_layers.shape

    with open(filename, "wb") as f:

        f.write(
            struct.pack(
                "<iii",
                height,
                width,
                objectives
            )
        )

        cost_layers.astype(
            np.float32
        ).tofile(f)


# ---------------------------------------------------------
# Main processing
# ---------------------------------------------------------

def process_maps(
        maps_root,
        output_root,
        objectives=5,
        base_seed=12345):

    os.makedirs(
        output_root,
        exist_ok=True
    )

    map_files = sorted(
        glob.glob(
            os.path.join(
                maps_root,
                "**",
                "*.map"
            ),
            recursive=True
        )
    )

    print(f"Found {len(map_files)} maps")

    for idx, map_path in enumerate(map_files):

        traversable = load_movingai_map(
            map_path
        )

        layers = generate_cost_layers(
            traversable,
            objectives=objectives,
            seed=base_seed + idx
        )

        rel = os.path.relpath(
            map_path,
            maps_root
        )

        out_file = os.path.splitext(
            rel
        )[0] + ".cost"

        out_file = os.path.join(
            output_root,
            out_file
        )

        os.makedirs(
            os.path.dirname(out_file),
            exist_ok=True
        )

        save_binary_cost_map(
            out_file,
            layers
        )

        print(
            f"[{idx+1}/{len(map_files)}] "
            f"{out_file}"
        )


if __name__ == "__main__":

    import sys

    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)

    maps_root = os.path.join(project_root, "maps")
    output_root = os.path.join(project_root, "maps", "mo_costmaps")

    objectives = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    base_seed = int(sys.argv[2]) if len(sys.argv) > 2 else 12345

    process_maps(
        maps_root=maps_root,
        output_root=output_root,
        objectives=objectives,
        base_seed=base_seed
    )
