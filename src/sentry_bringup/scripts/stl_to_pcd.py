#!/usr/bin/env python3
"""
stl_to_pcd.py

将 STL 三角网格转换为 PCD 点云，用于 Point-LIO / KISS-Matcher 先验地图。
纯 numpy 实现，无需 open3d / trimesh / CloudCompare。

用法:
    python3 stl_to_pcd.py input.stl output.pcd [options]

示例:
    python3 stl_to_pcd.py field.stl scans.pcd --density 500 --zmin -0.1 --zmax 2.0
"""

import sys
import struct
import argparse
import numpy as np


def read_stl_binary(filepath):
    """读取二进制 STL 文件，返回顶点数组 [N, 3, 3]（每个三角形的 3 个顶点）"""
    with open(filepath, 'rb') as f:
        header = f.read(80)
        num_triangles = struct.unpack('<I', f.read(4))[0]

        triangles = np.zeros((num_triangles, 3, 3), dtype=np.float32)
        for i in range(num_triangles):
            data = f.read(50)  # normal(12) + v0(12) + v1(12) + v2(12) + attr(2)
            if len(data) < 50:
                break
            # 跳过法向量，读取 3 个顶点
            nx, ny, nz, x0, y0, z0, x1, y1, z1, x2, y2, z2 = struct.unpack('<12f', data[:48])
            triangles[i] = [[x0, y0, z0], [x1, y1, z1], [x2, y2, z2]]

    print(f"[STL] 读取完成: {num_triangles} 个三角形")
    return triangles


def sample_triangle(v0, v1, v2, num_points):
    """在单个三角形上均匀随机采样 num_points 个点"""
    r1 = np.sqrt(np.random.rand(num_points))
    r2 = np.random.rand(num_points)

    u = 1.0 - r1
    v = r1 * (1.0 - r2)
    w = r1 * r2

    points = (u[:, None] * v0 +
              v[:, None] * v1 +
              w[:, None] * v2)
    return points


def sample_mesh(triangles, density_per_m2=500):
    """在整个网格上采样点云
    density_per_m2: 每平方米采样点数
    """
    all_points = []
    total_area = 0.0

    for tri in triangles:
        v0, v1, v2 = tri
        # 计算三角形面积
        cross = np.cross(v1 - v0, v2 - v0)
        area = 0.5 * np.linalg.norm(cross)
        total_area += area

        if area < 1e-9:
            continue

        num_samples = max(1, int(area * density_per_m2))
        pts = sample_triangle(v0, v1, v2, num_samples)
        all_points.append(pts)

    if not all_points:
        return np.zeros((0, 3), dtype=np.float32)

    points = np.vstack(all_points)
    print(f"[采样] 总面积: {total_area:.2f} m², 采样点数: {len(points)}")
    return points.astype(np.float32)


def filter_points(points, x_range=None, y_range=None, z_min=None, z_max=None,
                  voxel_size=None):
    """过滤点云"""
    mask = np.ones(len(points), dtype=bool)

    if x_range is not None:
        mask &= (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1])
    if y_range is not None:
        mask &= (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1])
    if z_min is not None:
        mask &= (points[:, 2] >= z_min)
    if z_max is not None:
        mask &= (points[:, 2] <= z_max)

    points = points[mask]
    print(f"[过滤] 过滤后点数: {len(points)}")

    # 体素下采样（简单网格化）
    if voxel_size is not None and len(points) > 0:
        coords = np.floor(points / voxel_size).astype(np.int32)
        unique_coords, indices = np.unique(coords, axis=0, return_index=True)
        points = points[indices]
        print(f"[下采样] 体素大小 {voxel_size}m，下采样后点数: {len(points)}")

    return points


def write_pcd_ascii(points, filepath):
    """写入 ASCII 格式 PCD 文件（PCL 兼容）"""
    num_points = len(points)
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {num_points}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {num_points}
DATA ascii
"""
    with open(filepath, 'w') as f:
        f.write(header)
        for p in points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

    print(f"[PCD] 保存完成: {filepath} ({num_points} 点)")


def main():
    parser = argparse.ArgumentParser(
        description='STL to PCD converter for Point-LIO prior map')
    parser.add_argument('input_stl', help='输入 STL 文件路径')
    parser.add_argument('output_pcd', help='输出 PCD 文件路径')
    parser.add_argument('--density', type=float, default=500,
                        help='每平方米采样点数 (默认: 500)')
    parser.add_argument('--zmin', type=float, default=None,
                        help='最小 Z 高度过滤 (米)')
    parser.add_argument('--zmax', type=float, default=None,
                        help='最大 Z 高度过滤 (米)')
    parser.add_argument('--xmin', type=float, default=None)
    parser.add_argument('--xmax', type=float, default=None)
    parser.add_argument('--ymin', type=float, default=None)
    parser.add_argument('--ymax', type=float, default=None)
    parser.add_argument('--voxel', type=float, default=0.05,
                        help='体素下采样大小 (米, 默认: 0.05)')
    parser.add_argument('--no-voxel', action='store_true',
                        help='禁用体素下采样')

    args = parser.parse_args()

    # 读取 STL
    triangles = read_stl_binary(args.input_stl)

    # 采样
    points = sample_mesh(triangles, density_per_m2=args.density)

    # 过滤
    x_range = (args.xmin, args.xmax) if args.xmin is not None and args.xmax is not None else None
    y_range = (args.ymin, args.ymax) if args.ymin is not None and args.ymax is not None else None
    voxel_size = None if args.no_voxel else args.voxel

    points = filter_points(points,
                           x_range=x_range,
                           y_range=y_range,
                           z_min=args.zmin,
                           z_max=args.zmax,
                           voxel_size=voxel_size)

    if len(points) == 0:
        print("[错误] 过滤后点数为 0，请检查过滤参数")
        sys.exit(1)

    # 写入 PCD
    write_pcd_ascii(points, args.output_pcd)
    print("[完成] 先验地图已生成，可直接用于 KISS-Matcher")


if __name__ == '__main__':
    main()
