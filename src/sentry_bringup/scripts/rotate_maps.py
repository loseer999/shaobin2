#!/usr/bin/env python3
"""旋转 PCD 和 PGM/YAML 地图（顺时针 47°）

用法:
    python3 rotate_maps.py --pcd input.pcd --pgm input.pgm --yaml input.yaml --out_dir ./rotated
"""

import argparse
import numpy as np
import struct
from pathlib import Path


def rotate_pcd(input_path: str, output_path: str, angle_deg: float):
    """顺时针旋转 PCD 地图"""
    angle_rad = np.radians(angle_deg)
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)

    with open(input_path, 'rb') as f:
        # Read header
        header_lines = []
        data_format = None
        while True:
            line = f.readline().decode('ascii').strip()
            header_lines.append(line)
            if line.startswith('DATA'):
                data_format = line.split()[-1]
                break

        # Parse header
        header_dict = {}
        for line in header_lines:
            parts = line.split()
            if len(parts) >= 2:
                header_dict[parts[0]] = parts[1:]

        width = int(header_dict.get('WIDTH', ['0'])[0])
        height = int(header_dict.get('HEIGHT', ['0'])[0])
        points_count = width * height

        fields = header_dict.get('FIELDS', [])
        sizes = [int(x) for x in header_dict.get('SIZE', [])]
        types = header_dict.get('TYPE', [])
        counts = [int(x) for x in header_dict.get('COUNT', [])]

        # Read data
        if data_format == 'ascii':
            data = f.read().decode('ascii').strip().split('\n')
            points = []
            for line in data[:points_count]:
                vals = [float(x) for x in line.split()]
                points.append(vals)
            points = np.array(points, dtype=np.float64)
        elif data_format == 'binary':
            point_size = sum(sizes)
            buf = f.read(points_count * point_size)
            fmt_parts = []
            for t, s in zip(types, sizes):
                if t == 'F' and s == 4:
                    fmt_parts.append('f')
                elif t == 'F' and s == 8:
                    fmt_parts.append('d')
                elif t == 'I' and s == 4:
                    fmt_parts.append('i')
                elif t == 'U' and s == 4:
                    fmt_parts.append('I')
                else:
                    fmt_parts.append('f')
            fmt = '<' + ''.join(fmt_parts)
            points = []
            for i in range(points_count):
                offset = i * point_size
                vals = struct.unpack_from(fmt, buf, offset)
                points.append(list(vals))
            points = np.array(points, dtype=np.float64)
        else:
            raise ValueError(f"Unsupported DATA format: {data_format}")

    # Rotate XY coordinates (clockwise)
    x = points[:, 0]
    y = points[:, 1]
    points[:, 0] = x * cos_a + y * sin_a
    points[:, 1] = -x * sin_a + y * cos_a

    # Write output PCD
    with open(output_path, 'wb') as f:
        for line in header_lines:
            if line.startswith('VIEWPOINT'):
                # Update VIEWPOINT rotation (original * clockwise rotation)
                # For simplicity, keep identity and let software handle it
                continue
            f.write((line + '\n').encode('ascii'))

        if data_format == 'ascii':
            for p in points:
                line = ' '.join(str(v) for v in p) + '\n'
                f.write(line.encode('ascii'))
        elif data_format == 'binary':
            fmt_parts = []
            for t, s in zip(types, sizes):
                if t == 'F' and s == 4:
                    fmt_parts.append('f')
                elif t == 'F' and s == 8:
                    fmt_parts.append('d')
                elif t == 'I' and s == 4:
                    fmt_parts.append('i')
                elif t == 'U' and s == 4:
                    fmt_parts.append('I')
                else:
                    fmt_parts.append('f')
            fmt = '<' + ''.join(fmt_parts)
            for p in points:
                f.write(struct.pack(fmt, *p))

    print(f"[PCD] Rotated {points_count} points clockwise by {angle_deg}° → {output_path}")


def rotate_pgm_yaml(pgm_path: str, yaml_path: str, out_pgm: str, out_yaml: str, angle_deg: float):
    """顺时针旋转 PGM/YAML 地图"""
    try:
        from PIL import Image
    except ImportError:
        print("[ERROR] PIL not installed. Run: pip install Pillow")
        return

    angle_rad = np.radians(angle_deg)
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)

    # Read YAML
    import yaml
    with open(yaml_path, 'r') as f:
        yaml_data = yaml.safe_load(f)

    # Read PGM
    img = Image.open(pgm_path)
    orig_w, orig_h = img.size

    # Rotate image (PIL rotates counter-clockwise, so use negative for clockwise)
    rotated_img = img.rotate(-angle_deg, resample=Image.BICUBIC, expand=True)
    new_w, new_h = rotated_img.size

    # Save PGM
    rotated_img.save(out_pgm)

    # Update YAML origin
    orig_origin = np.array(yaml_data.get('origin', [0.0, 0.0, 0.0]))
    ox, oy = orig_origin[0], orig_origin[1]

    # Rotate origin (clockwise)
    new_ox = ox * cos_a + oy * sin_a
    new_oy = -ox * sin_a + oy * cos_a

    yaml_data['origin'] = [float(new_ox), float(new_oy), 0.0]

    # Update image path
    yaml_data['image'] = str(Path(out_pgm).name)

    with open(out_yaml, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False)

    print(f"[PGM] Rotated {orig_w}x{orig_h} → {new_w}x{new_h} clockwise by {angle_deg}° → {out_pgm}")
    print(f"[YAML] Updated origin: [{new_ox:.4f}, {new_oy:.4f}, 0.0] → {out_yaml}")


def main():
    parser = argparse.ArgumentParser(description='Rotate PCD and PGM maps clockwise')
    parser.add_argument('--pcd', required=True, help='Input PCD file')
    parser.add_argument('--pgm', required=True, help='Input PGM file')
    parser.add_argument('--yaml', required=True, help='Input YAML file')
    parser.add_argument('--out_dir', default='.', help='Output directory')
    parser.add_argument('--angle', type=float, default=47.0, help='Clockwise rotation angle in degrees')
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    pcd_name = Path(args.pcd).stem
    pgm_name = Path(args.pgm).stem

    out_pcd = out_dir / f"{pcd_name}_rot{args.angle}.pcd"
    out_pgm = out_dir / f"{pgm_name}_rot{args.angle}.pgm"
    out_yaml = out_dir / f"{pgm_name}_rot{args.angle}.yaml"

    rotate_pcd(args.pcd, str(out_pcd), args.angle)
    rotate_pgm_yaml(args.pgm, args.yaml, str(out_pgm), str(out_yaml), args.angle)


if __name__ == '__main__':
    main()
