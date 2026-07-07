#!/usr/bin/env python3
"""
KFS Top-3 All KFS1=3 Check
===========================
遍历所有不重复、合法、不对称的 KFS 摆放，检查是否存在某种摆放方式
使得 kfs2=2 的最优 3 条路径中，受影响的 kfs1 数量都等于 3。

如果存在这样的摆放，将网格和路径信息输出到终端。

Usage:
    cd mf_action_planner
    python3 test/kfs_top3_all_kfs1_3.py
"""

import sys
import os
import time
from multiprocessing import Pool, cpu_count

import numpy as np

# -- Allow import from the mf_action_planner package -----------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_DIR = os.path.dirname(_SCRIPT_DIR)
sys.path.insert(0, _PROJECT_DIR)

from test.kfs_placement_analysis import (
    enumerate_valid_grids,
    BatchDFSPlanner,
    HEIGHT_MAP_RED,
    METHOD,
    START_Y,
    GRID_ROWS,
    GRID_COLS,
    _INNER_TO_RC,
)

# ===================================================================
#  Multiprocessing Worker
# ===================================================================


def _check_worker(args):
    """对单个 grid 运行 DFS，检查 kfs2=2 最优 3 条路径是否全 kfs1=3。"""
    grid_list, _idx = args
    grid = np.array(grid_list, dtype=int)

    planner = BatchDFSPlanner(
        GRID_COLS, GRID_ROWS, grid,
        HEIGHT_MAP_RED,
        method=METHOD,
    )
    planner.plan_path(START_Y)

    paths_2 = planner.path[2]  # 已排序: (path, cost, kfs2, kfs1_list, turns)

    if len(paths_2) < 3:
        return {'idx': _idx, 'grid': grid_list, 'all_three_kfs1_3': False}

    top3 = paths_2[:3]
    kfs1_counts = [len(p[3]) for p in top3]
    all_three = all(c == 3 for c in kfs1_counts)

    return {
        'idx': _idx,
        'grid': grid_list,
        'all_three_kfs1_3': all_three,
        'paths_info': [
            {'cost': p[1], 'kfs1_count': len(p[3]), 'turns': p[4],
             'kfs1_list': p[3]}
            for p in top3
        ] if all_three else None,
    }


# ===================================================================
#  Grid Pretty-Printer
# ===================================================================

_CELL_CHAR = {0: '·', 1: '1', 2: '2', 3: 'F'}
_COLOR_RESET = '\033[0m'
_COLOR_RED = '\033[91m'
_COLOR_BLUE = '\033[94m'
_COLOR_GRAY = '\033[90m'

_CELL_COLOR = {
    0: _COLOR_GRAY,
    1: _COLOR_BLUE,
    2: _COLOR_RED,
    3: '\033[95m',
}


def print_grid(grid_list):
    """打印 6×3 网格到终端。Row 5 (exit) 在上，Row 0 (entry) 在下。"""
    print(f"     col 0   col 1   col 2")
    print(f"   ┌───────┬───────┬───────┐")
    for r in range(GRID_ROWS - 1, -1, -1):
        cells = []
        for c in range(GRID_COLS):
            val = int(grid_list[r][c])
            ch = _CELL_CHAR.get(val, '?')
            ch = f"{_CELL_COLOR.get(val, '')}{ch}{_COLOR_RESET}"
            cells.append(ch)
        print(f" row {r} │   {cells[0]}   │   {cells[1]}   │   {cells[2]}   │")
        if r > 0:
            print(f"   ├───────┼───────┼───────┤")
    print(f"   └───────┴───────┴───────┘")


def grid_summary(grid_list):
    """返回网格的文本摘要。"""
    inner = []
    for r, c in _INNER_TO_RC:
        inner.append(int(grid_list[r][c]))
    kfs2_positions = [i for i, v in enumerate(inner) if v == 2]
    kfs1_positions = [i for i, v in enumerate(inner) if v == 1]
    fake_pos = next(i for i, v in enumerate(inner) if v == 3)
    return (f"  KFS2 at inner indices: {kfs2_positions}\n"
            f"  KFS1 at inner indices: {kfs1_positions}\n"
            f"  Fake at inner index:   {fake_pos}")


# ===================================================================
#  Main
# ===================================================================


def main():
    print("=" * 60)
    print("  KFS Top-3 All KFS1=3 Check")
    print("=" * 60)
    print()
    print("问题：是否存在合法 KFS 摆放，使得 kfs2=2 的最优 3 条路径")
    print("      受影响的 kfs1 数量都等于 3？")
    print()

    # ---- Step 1: Enumerate ----
    print("[1/3] 枚举所有合法 KFS 摆放 ...")
    t0 = time.time()
    grids = list(enumerate_valid_grids())
    t1 = time.time()
    print(f"  共生成 {len(grids):,} 个不重复、合法、不对称的摆放 ({t1 - t0:.1f}s)")
    print()

    # ---- Step 2: Check each grid ----
    print(f"[2/3] 逐个检查 kfs2=2 最优 3 条路径 "
          f"(multiprocessing, {cpu_count()} cores) ...")
    print()

    worker_args = [(g.tolist(), i) for i, g in enumerate(grids)]

    t0 = time.time()
    matched_grids = []
    total = len(grids)

    with Pool() as pool:
        for i, result in enumerate(pool.imap_unordered(_check_worker, worker_args,
                                                       chunksize=50)):
            if result['all_three_kfs1_3']:
                matched_grids.append(result)
                print(f"  ★ 发现符合条件的摆放！ (目前已找到 {len(matched_grids)} 个)")

            if (i + 1) % 500 == 0 or (i + 1) == total:
                elapsed = time.time() - t0
                rate = (i + 1) / elapsed if elapsed > 0 else 0
                remaining = (total - i - 1) / rate if rate > 0 else 0
                print(f"    进度: {i+1:,}/{total:,}  "
                      f"({elapsed:.0f}s elapsed, ~{remaining:.0f}s remaining)")

    t1 = time.time()
    print(f"\n  检查完成，耗时 {t1 - t0:.0f}s "
          f"({(t1 - t0) / total * 1000:.1f}ms/grid)")
    print()

    # ---- Step 3: Report ----
    print("[3/3] 结果：")
    print()

    if matched_grids:
        print(f"{'='*60}")
        print(f"  结论：存在 {len(matched_grids)} 种摆放方式，")
        print(f"        其 kfs2=2 最优 3 条路径受影响的 kfs1 都等于 3！")
        print(f"{'='*60}")
        print()
        for i, item in enumerate(matched_grids):
            print(f"━━━ 摆放 #{i+1} (grid index: {item['idx']}) ━━━")
            print()
            print_grid(item['grid'])
            print()
            print(grid_summary(item['grid']))
            print()
            print(f"  kfs2=2 最优 3 条路径：")
            for j, pinfo in enumerate(item['paths_info']):
                print(f"    #{j+1}: cost={pinfo['cost']:.1f}, "
                      f"kfs1={pinfo['kfs1_count']}, "
                      f"turns={pinfo['turns']}, "
                      f"affected_kfs1={pinfo['kfs1_list']}")
            print()
    else:
        print(f"{'='*60}")
        print(f"  结论：所有 {total:,} 种合法摆放中，")
        print(f"        不存在 kfs2=2 最优 3 条路径全为 kfs1=3 的摆放方式。")
        print(f"{'='*60}")

    print()
    print("完成。")


if __name__ == '__main__':
    main()
