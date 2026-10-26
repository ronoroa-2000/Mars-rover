#!/usr/bin/env python3
"""Convert the Marsyard2026 terrain heightmap into a Nav2 map_server map.

Slope-thresholds the same heightmap Gazebo uses for collision/visual
(rover_description/models/dem/marsyard2026_terrain_hm_visual.png) so the
occupancy grid matches what the rover actually drives on, rather than a
hand-captured RViz screenshot.

Usage:
    python3 generate_occupancy_map.py
    python3 generate_occupancy_map.py --slope-threshold-deg 20
"""
import argparse
from pathlib import Path

import numpy as np
from PIL import Image

# Must match the <heightmap> block in rover_description/worlds/marsyard2026.sdf:
# <pos> gives the patch center (cx, cy), <size> gives side length and z-span.
DEFAULT_HEIGHTMAP = (
    Path(__file__).resolve().parents[2]
    / "rover_description" / "models" / "dem" / "marsyard2026_terrain_hm_visual.png"
)
DEFAULT_SIDE = 44.0
DEFAULT_SPAN = 2.254232
DEFAULT_CX = 0.190492
DEFAULT_CY = 14.019499


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--heightmap", type=Path, default=DEFAULT_HEIGHTMAP)
    ap.add_argument("--side", type=float, default=DEFAULT_SIDE)
    ap.add_argument("--span", type=float, default=DEFAULT_SPAN)
    ap.add_argument("--center-x", type=float, default=DEFAULT_CX)
    ap.add_argument("--center-y", type=float, default=DEFAULT_CY)
    # 25 deg is a common traversability ceiling for a rocker-bogie rover on
    # compacted sand; tune per your rover's actual climb limit.
    ap.add_argument("--slope-threshold-deg", type=float, default=25.0)
    ap.add_argument(
        "--out-dir", type=Path,
        default=Path(__file__).resolve().parent.parent / "maps",
    )
    ap.add_argument("--name", default="marsyard2026")
    args = ap.parse_args()

    im = Image.open(args.heightmap)
    raw = np.array(im).astype(np.float64)
    h, w = raw.shape
    if h != w:
        raise ValueError(f"expected a square heightmap, got {w}x{h}")

    elevation = raw / 65535.0 * args.span
    resolution = args.side / (w - 1)

    # Gazebo's heightmap row 0 is +Y (north); np.gradient's spacing is
    # orientation-independent for the magnitude used here, so no flip needed.
    dz_dy, dz_dx = np.gradient(elevation, resolution)
    slope_deg = np.degrees(np.arctan(np.hypot(dz_dx, dz_dy)))

    occupied = slope_deg > args.slope_threshold_deg
    grid = np.where(occupied, 0, 255).astype(np.uint8)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = args.out_dir / f"{args.name}.pgm"
    yaml_path = args.out_dir / f"{args.name}.yaml"

    Image.fromarray(grid, mode="L").save(pgm_path)

    origin_x = args.center_x - args.side / 2.0
    origin_y = args.center_y - args.side / 2.0
    yaml_path.write_text(
        f"image: {pgm_path.name}\n"
        f"resolution: {resolution:.6f}\n"
        f"origin: [{origin_x:.6f}, {origin_y:.6f}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )

    occ_pct = 100.0 * occupied.sum() / occupied.size
    print(f"wrote {pgm_path} ({w}x{h}, {resolution * 100:.1f} cm/cell)")
    print(f"wrote {yaml_path}")
    print(f"origin: ({origin_x:.3f}, {origin_y:.3f}), 0.0 rad")
    print(f"extent: x [{origin_x:.2f}, {origin_x + args.side:.2f}], "
          f"y [{origin_y:.2f}, {origin_y + args.side:.2f}]")
    print(f"occupied cells: {occ_pct:.1f}% (slope > {args.slope_threshold_deg} deg)")


if __name__ == "__main__":
    main()
