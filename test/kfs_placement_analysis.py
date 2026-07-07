#!/usr/bin/env python3
"""
KFS Placement Analysis
======================
Enumerate all valid KFS placements on a 6×3 grid, rank them by worst optimal
path cost for collecting exactly 2 and 3 KFS2, and generate a LaTeX file
visualizing the worst-30 placements for each scenario.

Constraints (per user specification):
    - Exactly 4 KFS2  (value 2)  — anywhere in the 12 inner cells (rows 1–4)
    - Exactly 3 KFS1  (value 1)  — only at boundary cells of the inner area
      (top row 1, bottom row 4, left col 0, right col 2 — 10 cells total)
    - Exactly 1 fake  (value 3)  — NOT in row 1
    - Remaining 4 cells are empty (value 0)

Usage:
    cd mf_action_planner
    python3 test/kfs_placement_analysis.py

Output:
    test/output/kfs_placement_analysis.tex   — LaTeX visualization
    test/output/results.json                 — intermediate results
"""

import sys
import os
import math
import json
import time
import itertools
from itertools import combinations
from multiprocessing import Pool, cpu_count

import numpy as np

# -- Allow import from the mf_action_planner package -----------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_DIR = os.path.dirname(_SCRIPT_DIR)
sys.path.insert(0, _PROJECT_DIR)

from core.dfs import DFSPlanner

# ===================================================================
#  Constants
# ===================================================================

HEIGHT_MAP_RED = np.array([
    [  0,   0,   0],
    [400, 200, 400],
    [200, 400, 600],
    [400, 600, 400],
    [200, 400, 200],
    [  0,   0,   0],
], dtype=int)

METHOD = 1          # 1 = unidirectional
# 代价模型：高度差+抓取（上+200:2.5s 上+400:4.0s 下-200:3.5s 下-400:5.0s 抓取:4.0s）
START_Y = [0, 1, 2]
GRID_ROWS = 6
GRID_COLS = 3

OUTPUT_DIR = os.path.join(_SCRIPT_DIR, 'output')

# Inner-index → (row, col)  (12 cells, rows 1–4, cols 0–2)
_INNER_TO_RC = [
    (1, 0), (1, 1), (1, 2),
    (2, 0), (2, 1), (2, 2),
    (3, 0), (3, 1), (3, 2),
    (4, 0), (4, 1), (4, 2),
]

# Cells where KFS1 (value 1) is allowed: all boundary cells of the inner 4×3 area
# (top row 1, bottom row 4, left col 0, right col 2 — the "four edges")
# inner indices: 0=(1,0), 1=(1,1), 2=(1,2), 3=(2,0), 5=(2,2),
#                6=(3,0), 8=(3,2), 9=(4,0), 10=(4,1), 11=(4,2)
_KFS1_CAPABLE = [0, 1, 2, 3, 5, 6, 8, 9, 10, 11]

# Cells where fake KFS (value 3) is allowed: rows 2-4, all cols
# inner indices: 3,4,5,6,7,8,9,10,11
_FAKE_CAPABLE = [3, 4, 5, 6, 7, 8, 9, 10, 11]


# ===================================================================
#  Grid Enumeration
# ===================================================================

def _grid_canonical_key(grid):
    """Return a canonical bytes key that is identical for column-0↔2 symmetric grids."""
    swapped = grid.copy()
    swapped[:, [0, 2]] = swapped[:, [2, 0]]
    key_g = grid.tobytes()
    key_s = swapped.tobytes()
    return key_g if key_g <= key_s else key_s


def enumerate_valid_grids():
    """Generator yielding valid (6,3) grids, with column-symmetric duplicates removed.

    Constraints: exactly 4 KFS2 (anywhere in 12 inner cells),
    3 KFS1 (only outermost), 1 fake KFS (not row 1), rest empty.

    Symmetry rule: two grids that differ only by swapping column 0 ↔ column 2
    are considered the same; only the canonical one is yielded.
    """
    all_12 = list(range(12))
    base = np.zeros((GRID_ROWS, GRID_COLS), dtype=int)
    seen = set()

    for kfs2_positions in combinations(all_12, 4):
        kfs2_set = set(kfs2_positions)

        # KFS1: exactly 3 from outermost, excluding KFS2 cells
        kfs1_candidates = [i for i in _KFS1_CAPABLE if i not in kfs2_set]
        if len(kfs1_candidates) < 3:
            continue

        for kfs1_positions in combinations(kfs1_candidates, 3):
            kfs1_set = set(kfs1_positions)
            occupied = kfs2_set | kfs1_set

            # Fake: exactly 1 from fake-capable, excluding occupied cells
            fake_candidates = [i for i in _FAKE_CAPABLE if i not in occupied]
            if len(fake_candidates) < 1:
                continue

            for fake_pos in fake_candidates:
                # Build inner array
                inner = np.zeros(12, dtype=int)
                for i in kfs2_set:
                    inner[i] = 2
                for i in kfs1_set:
                    inner[i] = 1
                inner[fake_pos] = 3

                # Build full 6×3 grid
                grid = base.copy()
                for i, (r, c) in enumerate(_INNER_TO_RC):
                    grid[r, c] = inner[i]

                # Dedup symmetric pairs (col 0 ↔ col 2 swap)
                key = _grid_canonical_key(grid)
                if key in seen:
                    continue
                seen.add(key)

                yield grid


# ===================================================================
#  Batch DFS Planner
# ===================================================================

class BatchDFSPlanner(DFSPlanner):
    """DFSPlanner subclass that evaluates kfs2_indicated=2, 3, and 4."""

    def plan_path(self, sty):
        self.path = [[], [], [], [], []]
        for y in sty:
            for i in (2, 3, 4):
                self.visited = np.zeros((self.GRID_ROWS, self.GRID_COLS), dtype=bool)
                self.cnt_path.clear()
                self.eval_path.clear()
                self.kfs2_indicated = i
                self._dfs(0, y, 0)
        return self.path


# ===================================================================
#  Multiprocessing Worker
# ===================================================================

def _plan_worker(args):
    grid_list, _idx = args
    grid = np.array(grid_list, dtype=int)

    planner = BatchDFSPlanner(
        GRID_COLS, GRID_ROWS, grid,
        HEIGHT_MAP_RED,
        method=METHOD,
    )
    planner.plan_path(START_Y)

    best_2 = planner.path[2][0] if planner.path[2] else None
    best_3 = planner.path[3][0] if planner.path[3] else None
    best_4 = planner.path[4][0] if planner.path[4] else None

    # kfs1_affected stores [nowX-1, nowY]; convert to actual grid (r, c)
    def _actual_kfs1_positions(best):
        if best is None:
            return []
        return [[p[0] + 1, p[1]] for p in best[3]]

    return {
        'grid': grid_list,
        'cost_2': best_2[1] if best_2 else float('inf'),
        'cost_3': best_3[1] if best_3 else float('inf'),
        'cost_4': best_4[1] if best_4 else float('inf'),
        'kfs1_2': len(best_2[3]) if best_2 else 0,
        'kfs1_3': len(best_3[3]) if best_3 else 0,
        'kfs1_4': len(best_4[3]) if best_4 else 0,
        'kfs1_affected_2': _actual_kfs1_positions(best_2),
        'kfs1_affected_3': _actual_kfs1_positions(best_3),
        'kfs1_affected_4': _actual_kfs1_positions(best_4),
        'steps_2': len(best_2[0]) if best_2 else 0,
        'steps_3': len(best_3[0]) if best_3 else 0,
        'steps_4': len(best_4[0]) if best_4 else 0,
    }


# ===================================================================
#  Ranking
# ===================================================================

def rank_results(results, cost_key='cost_2', kfs1_key='kfs1_2', top_n=30):
    feasible = [r for r in results if r[cost_key] < float('inf')]
    feasible.sort(key=lambda r: (r[cost_key], r[kfs1_key]), reverse=True)
    # Return shallow copies so label assignment in generate_tex doesn't
    # clobber shared dicts when a grid appears in both worst_2 and worst_3.
    return [dict(r) for r in feasible[:top_n]]


def print_ranking_summary(worst_list, label, cost_key='cost_2', kfs1_key='kfs1_2'):
    print(f"\n{'='*70}")
    print(f"  Worst 30 — {label}")
    print(f"{'='*70}")
    print(f"{'Rank':<6} {'Cost':<8} {'KFS1 aff.':<10} {'Steps':<8}")
    print(f"{'-'*32}")
    for i, r in enumerate(worst_list):
        suffix = cost_key.split('_', 1)[1]
        steps_key = f'steps_{suffix}'
        print(f"{i+1:<6} {r[cost_key]:<8.1f} {r[kfs1_key]:<10} {r[steps_key]:<8}")


# ===================================================================
#  LaTeX Generation
# ===================================================================

_CELL_COLORS = {0: 'white', 1: 'blue!30', 2: 'red!30', 3: 'black!15'}
_CELL_LABELS = {0: '0', 1: '1', 2: '2', 3: 'f'}


def _tikz_grid(grid_list, affected_kfs1=None):
    """Return TikZ commands for a 6×3 grid.

    Row 5 (exit) at top, row 0 (entry) at bottom.
    Column 0 on the **right** (col 2 on left).

    Parameters
    ----------
    affected_kfs1 : list of [r, c] or None
        KFS1 cells that were on the optimal path (shown in darker blue).
    """
    affected_set = set()
    if affected_kfs1:
        affected_set = {(int(r), int(c)) for r, c in affected_kfs1}

    lines = []
    lines.append(r'\begin{tikzpicture}[scale=0.55]')

    # Column headers  (c=0 rightmost → tikz-x=2, c=2 leftmost → tikz-x=0)
    for c in range(GRID_COLS):
        tikz_x = GRID_COLS - 1 - c
        lines.append(r'  \node[font=\tiny\bfseries] at ({x}+0.5, 6.6) {{{c}}};'
                     .format(x=tikz_x, c=c))

    # Row labels  (r=5 at top tikz-y=5, r=0 at bottom tikz-y=0)
    for r in range(GRID_ROWS):
        lines.append(r'  \node[font=\tiny\bfseries, anchor=east] at (-0.3, {y}+0.5) {{{r}}};'
                     .format(y=r, r=r))

    # Cells
    for r in range(GRID_ROWS):
        tikz_y = r
        for c in range(GRID_COLS):
            tikz_x = GRID_COLS - 1 - c   # col 0 → rightmost
            val = int(grid_list[r][c])
            if val == 1 and (r, c) in affected_set:
                color = 'blue!60'   # darker: affected KFS1
            else:
                color = _CELL_COLORS.get(val, 'white')
            label = _CELL_LABELS.get(val, '?')
            lines.append(
                r'  \fill[{color}] ({x},{y}) rectangle ++(1,1);'
                .format(color=color, x=tikz_x, y=tikz_y)
            )
            lines.append(
                r'  \draw ({x},{y}) rectangle ++(1,1);'
                .format(x=tikz_x, y=tikz_y)
            )
            lines.append(
                r'  \node[font=\footnotesize] at ({x}+0.5, {y}+0.5) {{{label}}};'
                .format(x=tikz_x, y=tikz_y, label=label)
            )
    lines.append(r'\end{tikzpicture}')
    return '\n'.join(lines)


def _scheme_block(rank_info, cost, kfs1_count, affected_kfs1, target_kfs2):
    """Return LaTeX for a single scheme (grid + one-line annotation)."""
    grid_list = rank_info['grid']
    label = rank_info.get('label', '')
    tikz = _tikz_grid(grid_list, affected_kfs1)

    # One-line annotation, e.g.  "#01 cost=28.0, k1=2"
    annotation = f'{label} cost={cost:.1f}, k1={kfs1_count}'

    return (
        r'\begin{minipage}[t]{0.32\textwidth}' + '\n'
        r'\centering' + '\n'
        r'\textbf{' + annotation + r'}\\[2mm]' + '\n'
        + tikz + '\n'
        r'\end{minipage}'
    )


def generate_tex(worst_2, worst_3, worst_4, output_path):
    """Generate the full LaTeX file.

    Layout: 5 rows × 3 cols per page = 15 schemes/page.
    30 schemes = 2 pages per KFS2 scenario.
    """
    # Build rank labels
    for i, r in enumerate(worst_2):
        r['label'] = f'\\#{{{i+1:02d}}}'
    for i, r in enumerate(worst_3):
        r['label'] = f'\\#{{{i+1:02d}}}'
    for i, r in enumerate(worst_4):
        r['label'] = f'\\#{{{i+1:02d}}}'

    def _write_section(lines_out, items, cost_key, kfs1_key, affected_key, target_kfs2):
        """Write one section (page of 15 items, 5 rows × 3 cols)."""
        for page_start in range(0, len(items), 15):
            page_items = items[page_start:page_start + 15]
            for row_idx in range(0, len(page_items), 3):
                a = page_items[row_idx]
                b = page_items[row_idx + 1] if row_idx + 1 < len(page_items) else None
                c = page_items[row_idx + 2] if row_idx + 2 < len(page_items) else None

                lines_out.append(r'\vspace{2mm}')
                lines_out.append(r'\noindent')
                lines_out.append(_scheme_block(a, a[cost_key], a[kfs1_key],
                                               a.get(affected_key, []), target_kfs2))

                if b:
                    lines_out.append(r'\hfill')
                    lines_out.append(_scheme_block(b, b[cost_key], b[kfs1_key],
                                                   b.get(affected_key, []), target_kfs2))
                else:
                    lines_out.append(r'\hfill')
                    lines_out.append(r'\begin{minipage}[t]{0.32\textwidth}\centering~\end{minipage}')

                if c:
                    lines_out.append(r'\hfill')
                    lines_out.append(_scheme_block(c, c[cost_key], c[kfs1_key],
                                                   c.get(affected_key, []), target_kfs2))
                else:
                    lines_out.append(r'\hfill')
                    lines_out.append(r'\begin{minipage}[t]{0.32\textwidth}\centering~\end{minipage}')

                lines_out.append(r'\vspace{4mm}')

            if page_start + 15 < len(items):
                lines_out.append(r'\newpage')

    lines = []
    lines.append(r"""\documentclass[a4paper]{article}
\usepackage[utf8]{inputenc}
\usepackage{tikz}
\usepackage[margin=1cm]{geometry}
\usepackage{multicol}

\begin{document}
""")

    # ---- Section 1: Worst-30 for 2 KFS2 ----
    lines.append(r'\section*{Worst 30 KFS Placements --- 2 KFS2 Collection}')
    lines.append(r'\vspace{1mm}')
    _write_section(lines, worst_2, 'cost_2', 'kfs1_2', 'kfs1_affected_2', 2)

    # ---- Section 2: Worst-30 for 3 KFS2 ----
    lines.append(r'\newpage')
    lines.append(r'\section*{Worst 30 KFS Placements --- 3 KFS2 Collection}')
    lines.append(r'\vspace{1mm}')
    _write_section(lines, worst_3, 'cost_3', 'kfs1_3', 'kfs1_affected_3', 3)

    # ---- Section 3: Worst-30 for 4 KFS2 ----
    lines.append(r'\newpage')
    lines.append(r'\section*{Worst 30 KFS Placements --- 4 KFS2 Collection}')
    lines.append(r'\vspace{1mm}')
    _write_section(lines, worst_4, 'cost_4', 'kfs1_4', 'kfs1_affected_4', 4)

    lines.append(r'\end{document}')

    with open(output_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))
        f.write('\n')

    print(f"  TeX written to: {output_path}")


# ===================================================================
#  Main
# ===================================================================

def main():
    print("=" * 60)
    print("  KFS Placement Analysis")
    print("=" * 60)

    # ---- Step 1: Enumerate ----
    print("\n[1/4] Enumerating valid KFS placements ...")
    t0 = time.time()
    grids = list(enumerate_valid_grids())
    t1 = time.time()
    print(f"  Generated {len(grids):,} valid grids in {t1 - t0:.1f}s")

    # Quick sanity check
    for g in grids[:1]:
        assert np.count_nonzero(g[1:5, :] == 2) == 4, "KFS2 count != 4"
        assert np.count_nonzero(g[1:5, :] == 1) == 3, "KFS1 count != 3"
        assert np.count_nonzero(g[1:5, :] == 3) == 1, "fake count != 1"
    print(f"  Verified: each grid has 4×KFS2 + 3×KFS1 + 1×fake + 4×empty = 12 cells")

    # ---- Step 2: Plan ----
    print(f"\n[2/4] Planning paths for {len(grids):,} grids "
          f"(multiprocessing, {cpu_count()} cores) ...")

    worker_args = [(g.tolist(), i) for i, g in enumerate(grids)]

    t0 = time.time()
    results = []
    with Pool() as pool:
        for i, result in enumerate(pool.imap_unordered(_plan_worker, worker_args,
                                                       chunksize=50)):
            results.append(result)
            if (i + 1) % 500 == 0 or (i + 1) == len(grids):
                elapsed = time.time() - t0
                rate = (i + 1) / elapsed if elapsed > 0 else 0
                remaining = (len(grids) - i - 1) / rate if rate > 0 else 0
                print(f"    Progress: {i+1:,}/{len(grids):,} grids "
                      f"({elapsed:.0f}s elapsed, ~{remaining:.0f}s remaining)")
    t1 = time.time()
    print(f"  Planning complete in {t1 - t0:.0f}s "
          f"({(t1 - t0) / len(grids) * 1000:.1f}ms/grid)")

    # ---- Step 3: Rank ----
    print("\n[3/4] Ranking results ...")

    feasible_2 = sum(1 for r in results if r['cost_2'] < float('inf'))
    feasible_3 = sum(1 for r in results if r['cost_3'] < float('inf'))
    feasible_4 = sum(1 for r in results if r['cost_4'] < float('inf'))
    print(f"  Feasible 2-KFS2: {feasible_2}/{len(results)} "
          f"({100 * feasible_2 / len(results):.1f}%)")
    print(f"  Feasible 3-KFS2: {feasible_3}/{len(results)} "
          f"({100 * feasible_3 / len(results):.1f}%)")
    print(f"  Feasible 4-KFS2: {feasible_4}/{len(results)} "
          f"({100 * feasible_4 / len(results):.1f}%)")

    worst_2 = rank_results(results, 'cost_2', 'kfs1_2', top_n=30)
    worst_3 = rank_results(results, 'cost_3', 'kfs1_3', top_n=30)
    worst_4 = rank_results(results, 'cost_4', 'kfs1_4', top_n=30)

    print_ranking_summary(worst_2, "2-KFS2 collection", 'cost_2', 'kfs1_2')
    print_ranking_summary(worst_3, "3-KFS2 collection", 'cost_3', 'kfs1_3')
    print_ranking_summary(worst_4, "4-KFS2 collection", 'cost_4', 'kfs1_4')

    # ---- Step 4: Save & TeX ----
    print("\n[4/4] Saving results and generating TeX ...")
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    json_path = os.path.join(OUTPUT_DIR, 'results.json')
    save_data = {
        'worst_30_for_2': worst_2,
        'worst_30_for_3': worst_3,
        'worst_30_for_4': worst_4,
        'total_grids': len(grids),
        'feasible_2': feasible_2,
        'feasible_3': feasible_3,
        'feasible_4': feasible_4,
    }
    with open(json_path, 'w') as f:
        json.dump(save_data, f, indent=2, ensure_ascii=False)
    print(f"  JSON saved to: {json_path}")

    tex_path = os.path.join(OUTPUT_DIR, 'kfs_placement_analysis.tex')
    generate_tex(worst_2, worst_3, worst_4, tex_path)

    print(f"\n{'='*60}")
    print(f"  Done!")
    print(f"  Compile TeX:  pdflatex {tex_path}")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()
