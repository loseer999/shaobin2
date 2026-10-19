#!/bin/bash
# =====================================================================
# PCD 转 PGM/YAML 脚本
# ---------------------------------------------------------------------
# 用法: ./pcd_to_pgm.sh
#   1. 自动列出 src/point_lio/PCD/ 中的 PCD 文件
#   2. 输入文件名后回车转换
#   3. 输出到 src/pgm/ 目录
# =====================================================================

set -e

WORKSPACE_ROOT="/home/cubot/NAV"
PCD_DIR="${WORKSPACE_ROOT}/src/pcd/PCD"
OUT_DIR="${WORKSPACE_ROOT}/src/pgm"

# 检验并创建输出目录
if [ ! -d "$OUT_DIR" ]; then
    echo "[INFO] 创建输出目录: ${OUT_DIR}"
    mkdir -p "$OUT_DIR"
fi

# 检验 PCD 输入目录是否存在
if [ ! -d "$PCD_DIR" ]; then
    echo "[ERROR] PCD 输入目录不存在: ${PCD_DIR}"
    echo "        请确认已运行 mapping_bringup.sh 完成建图"
    exit 1
fi

echo "=============================================="
echo "  PCD → PGM/YAML 转换"
echo "  输入目录: ${PCD_DIR}"
echo "  输出目录: ${OUT_DIR}"
echo "=============================================="

# 列出 PCD 文件
echo ""
echo "可用 PCD 文件:"
ls -1 "${PCD_DIR}"/*.pcd 2>/dev/null | while read -r f; do
    name=$(basename "$f")
    size=$(du -h "$f" | cut -f1)
    echo "  • ${name} (${size})"
done

echo ""
read -rp "输入 PCD 文件名 (不含路径, 如 scans.pcd): " PCD_NAME

PCD_PATH="${PCD_DIR}/${PCD_NAME}"
if [ ! -f "$PCD_PATH" ]; then
    echo "[ERROR] 文件不存在: ${PCD_PATH}"
    exit 1
fi

echo ""
echo ">>> 转换中: ${PCD_NAME} ..."

export PCD_PATH_OUT="$PCD_PATH"
export OUT_DIR_OUT="$OUT_DIR"

python3 << 'PYEOF'
import numpy as np
import struct
import os
from pathlib import Path

pcd_path = os.environ['PCD_PATH_OUT']
out_dir = os.environ['OUT_DIR_OUT']
resolution = 0.05  # 栅格分辨率 5cm

# ---------- 读取 PCD ----------
with open(pcd_path, 'rb') as f:
    header_lines = []
    while True:
        line = f.readline().decode('ascii').strip()
        header_lines.append(line)
        if line.startswith('DATA'):
            break
    
    header_dict = {}
    for line in header_lines:
        parts = line.split()
        if len(parts) >= 2:
            header_dict[parts[0]] = parts[1:]
    
    fields = header_dict.get('FIELDS', [])
    sizes = [int(x) for x in header_dict.get('SIZE', [])]
    types = header_dict.get('TYPE', [])
    counts = [int(x) for x in header_dict.get('COUNT', [])]
    width = int(header_dict.get('WIDTH', ['0'])[0])
    height = int(header_dict.get('HEIGHT', ['0'])[0])
    points_count = width * height
    data_format = header_dict.get('DATA', [''])[0]
    
    # 找到 xyz 的索引
    x_idx = fields.index('x')
    y_idx = fields.index('y')
    z_idx = fields.index('z') if 'z' in fields else None
    
    # 计算每个字段的偏移
    offsets = []
    offset = 0
    for field, size, count in zip(fields, sizes, counts):
        offsets.append(offset)
        offset += size * count
    point_size = offset
    
    # 读取数据
    if data_format == 'ascii':
        data = f.read().decode('ascii').strip().split('\n')
        points = []
        for line in data[:points_count]:
            vals = [float(x) for x in line.split()]
            points.append(vals)
        points = np.array(points, dtype=np.float64)
    elif data_format == 'binary':
        buf = f.read(points_count * point_size)
        
        # 用 numpy.frombuffer 解析 (避免 struct 格式过长报错)
        np_type_map = {
            ('F', 4): np.float32, ('F', 8): np.float64,
            ('D', 8): np.float64,
            ('U', 1): np.uint8, ('U', 2): np.uint16, ('U', 4): np.uint32,
            ('I', 1): np.int8, ('I', 2): np.int16, ('I', 4): np.int32,
        }
        
        field_dtypes = []
        for i, (field, t, s, c) in enumerate(zip(fields, types, sizes, counts)):
            np_type = np_type_map.get((t.upper(), s))
            if np_type is None:
                raise ValueError(f"不支持的类型 {t}{s}")
            if c > 1:
                field_dtypes.append((f'f{i}', np_type, (c,)))
            else:
                field_dtypes.append((f'f{i}', np_type))
        
        points = np.frombuffer(buf, dtype=np.dtype(field_dtypes))
    else:
        raise ValueError(f"不支持的 DATA 格式: {data_format}")

x = points[f'f{x_idx}']
y = points[f'f{y_idx}']
z = points[f'f{z_idx}'] if z_idx is not None else None

# ---------- Z轴过滤 (-0.2 ~ 0.6m) ----------
Z_MIN = -0.600
Z_MAX = 0.80
if z is not None:
    mask = (z >= Z_MIN) & (z <= Z_MAX)
    x = x[mask]
    y = y[mask]
    print(f"   Z过滤: {Z_MIN}~{Z_MAX}m, 保留 {len(x)}/{len(mask)} 点")

# ---------- 计算边界框 ----------
min_x, max_x = x.min(), x.max()
min_y, max_y = y.min(), y.max()

# 添加边距
margin = 0.5
min_x -= margin
min_y -= margin
max_x += margin
max_y += margin

width_m = max_x - min_x
height_m = max_y - min_y
cols = int(np.ceil(width_m / resolution)) + 1
rows = int(np.ceil(height_m / resolution)) + 1

# ---------- 创建栅格地图 ----------
# Nav2 标准: 255=白色(自由), 0=黑色(障碍物), 205=灰色(未知)
grid = np.full((rows, cols), 255, dtype=np.uint8)  # 先全部设为自由空间

for xi, yi in zip(x, y):
    col = int((xi - min_x) / resolution)
    row = int((yi - min_y) / resolution)
    if 0 <= row < rows and 0 <= col < cols:
        grid[row, col] = 0  # 有点的地方 = 黑色(障碍物)

# 翻转 Y 轴 (PGM 原点在左上, 地图原点在左下)
grid = np.flipud(grid)

# ---------- 保存 PGM ----------
pcd_name = Path(pcd_path).stem
pgm_path = Path(out_dir) / f"{pcd_name}.pgm"
yaml_path = Path(out_dir) / f"{pcd_name}.yaml"

with open(pgm_path, 'wb') as f:
    f.write(b'P5\n')
    f.write(f'{cols} {rows}\n'.encode())
    f.write(b'255\n')
    f.write(grid.tobytes())

# ---------- 保存 YAML ----------
yaml_content = f"""image: {pgm_path.name}
resolution: {resolution}
origin: [{min_x:.6f}, {min_y:.6f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
with open(yaml_path, 'w') as f:
    f.write(yaml_content)

print(f"✅ PGM:  {pgm_path}")
print(f"✅ YAML: {yaml_path}")
print(f"   边界: x=[{min_x:.2f}, {max_x:.2f}], y=[{min_y:.2f}, {max_y:.2f}]")
print(f"   尺寸: {cols}x{rows} 像素, 分辨率={resolution}m")

PYEOF

echo ""
echo "=============================================="
echo "✅ 转换完成"
echo "   输出目录: ${OUT_DIR}"
ls -lh "${OUT_DIR}"/*.pgm "${OUT_DIR}"/*.yaml 2>/dev/null
echo "=============================================="
