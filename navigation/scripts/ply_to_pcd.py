#!/usr/bin/env python3
"""Convert a PLY point cloud (e.g. from rtabmap-export) to a binary XYZ PCD.

Only x/y/z are kept, which is all pcl_ros/pcd_to_pointcloud needs to show
the map in RViz.

Usage: ply_to_pcd.py input.ply output.pcd
"""
import sys

import numpy as np

PLY_TYPES = {
    'char': 'i1', 'int8': 'i1', 'uchar': 'u1', 'uint8': 'u1',
    'short': 'i2', 'int16': 'i2', 'ushort': 'u2', 'uint16': 'u2',
    'int': 'i4', 'int32': 'i4', 'uint': 'u4', 'uint32': 'u4',
    'float': 'f4', 'float32': 'f4', 'double': 'f8', 'float64': 'f8',
}


def read_ply_xyz(path):
    with open(path, 'rb') as f:
        if f.readline().strip() != b'ply':
            raise ValueError(f'{path} is not a PLY file')
        fmt, count, props, in_vertex = None, 0, [], False
        while True:
            line = f.readline().decode('ascii').strip()
            if line == 'end_header':
                break
            parts = line.split()
            if parts[0] == 'format':
                fmt = parts[1]
            elif parts[0] == 'element':
                in_vertex = parts[1] == 'vertex'
                if in_vertex:
                    count = int(parts[2])
            elif parts[0] == 'property' and in_vertex:
                if parts[1] == 'list':
                    raise ValueError('list properties on vertices are not supported')
                props.append((parts[2], PLY_TYPES[parts[1]]))

        if fmt == 'ascii':
            data = np.loadtxt(f, max_rows=count, ndmin=2)
            names = [n for n, _ in props]
            return data[:, [names.index(a) for a in 'xyz']].astype(np.float32)

        endian = '<' if fmt == 'binary_little_endian' else '>'
        dtype = np.dtype([(n, endian + t) for n, t in props])
        vertices = np.fromfile(f, dtype=dtype, count=count)
        return np.stack([vertices[a] for a in 'xyz'], axis=1).astype(np.float32)


def write_pcd_xyz(path, xyz):
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        f'WIDTH {len(xyz)}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(xyz)}\n'
        'DATA binary\n'
    )
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(np.ascontiguousarray(xyz, dtype='<f4').tobytes())
    return len(xyz)


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    n = write_pcd_xyz(sys.argv[2], read_ply_xyz(sys.argv[1]))
    print(f'Wrote {n} points to {sys.argv[2]}')


if __name__ == '__main__':
    main()
