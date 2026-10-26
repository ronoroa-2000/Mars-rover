#!/usr/bin/env python3
"""Sample a triangle-mesh OBJ into a voxel-downsampled XYZ PCD.

The OBJ is read as-is, so the output stays in the mesh's own frame. For the
Gazebo world that is the world frame (the terrain model sits at pose 0), which
makes the PCD usable directly as a scan-matching map.

Usage: mesh_to_pcd.py input.obj output.pcd [voxel_size_m] [points_per_m2]
"""
import sys

import numpy as np


def read_obj(path):
    verts, faces = [], []
    with open(path) as f:
        for line in f:
            if line.startswith('v '):
                verts.append(line.split()[1:4])
            elif line.startswith('f '):
                idx = [int(tok.split('/')[0]) for tok in line.split()[1:]]
                for i in range(1, len(idx) - 1):
                    faces.append((idx[0], idx[i], idx[i + 1]))
    verts = np.asarray(verts, dtype=np.float64)
    faces = np.asarray(faces, dtype=np.int64)
    # OBJ indices are 1-based; negative indices count from the end.
    faces = np.where(faces > 0, faces - 1, faces + len(verts))
    return verts, faces


def sample_surface(verts, faces, points_per_m2, rng):
    tri = verts[faces]
    area = 0.5 * np.linalg.norm(
        np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    n = int(area.sum() * points_per_m2)
    pick = rng.choice(len(faces), size=n, p=area / area.sum())
    u, v = rng.random(n), rng.random(n)
    flip = u + v > 1.0
    u[flip], v[flip] = 1.0 - u[flip], 1.0 - v[flip]
    t = tri[pick]
    return t[:, 0] + u[:, None] * (t[:, 1] - t[:, 0]) + v[:, None] * (t[:, 2] - t[:, 0])


def voxel_downsample(points, voxel):
    keys = np.floor(points / voxel).astype(np.int64)
    _, inverse, counts = np.unique(
        keys, axis=0, return_inverse=True, return_counts=True)
    inverse = inverse.ravel()
    sums = np.zeros((len(counts), 3))
    np.add.at(sums, inverse, points)
    return sums / counts[:, None]


def write_pcd(path, points):
    pts = points.astype('<f4')
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
        f'WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(pts)}\nDATA binary\n')
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(pts.tobytes())


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    src, dst = sys.argv[1], sys.argv[2]
    voxel = float(sys.argv[3]) if len(sys.argv) > 3 else 0.2
    density = float(sys.argv[4]) if len(sys.argv) > 4 else 50.0

    verts, faces = read_obj(src)
    print(f'{len(verts)} vertices, {len(faces)} triangles')
    points = sample_surface(verts, faces, density, np.random.default_rng(0))
    points = voxel_downsample(points, voxel)
    write_pcd(dst, points)
    print(f'{len(points)} points -> {dst}')
    print('min', points.min(0).round(2), 'max', points.max(0).round(2))


if __name__ == '__main__':
    main()
