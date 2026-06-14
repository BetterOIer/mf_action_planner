"""
DFS路径规划算法核心实现

逻辑：使用DFS遍历每一条路径。

可行性：
1. 首排有kfs2一定要取，不去取的路径不考虑。
2. 第一梯队：kfs2在必经之路上的一定要取
   第二梯队：去掉第一梯队的kfs2，到达某位置时，朝向的地方刚好有的kfs2
   第三梯队：去掉第一、二梯队的kfs2，到达某位置时，通过转向能拿到的kfs2
3. 必经之路上穿过了所有4个kfs2的路径不考虑。(默认R2不具备捡起kfs2扔到一边的功能，捡起了一定拿下，而R2上放不下4个kfs2)
4. 拿到小于2个kfs2的路径不考虑。

路径评估：

第一优先：路径的代价=移动代价+转向代价+精细调整+捡起kfs2代价
第二优先：受影响的kfs1数量（越少越好）
第三优先：取到kfs2的数量 2个优先于3个


"""

import math

import numpy as np


class DFSPlanner:
    def __init__(self, grid_cols, grid_rows, grid, kfs_grid_height, kfs2_preferred, method=1, logger=None):
        self.logger = logger
        self.GRID_ROWS = grid_rows
        self.GRID_COLS = grid_cols
        self.DIRECTIONS = [(1, 0, 0), (0, 1, math.pi / 2), (0, -1, -math.pi / 2)]

        self.method = method  # 1=uni, 0=omni

        self.GRID = grid
        self.kfs_grid_height = kfs_grid_height
        self.visited = np.zeros((self.GRID_ROWS, self.GRID_COLS), dtype=bool)
        self.kfs2_indicated=0
        self.path = [[], [], [], [], []]
        self.cnt_path = []
        self.eval_path = []

        self.MOVE_COST = 2
        self.TURN_COST = 1
        self.FETCH_KFS2_COST = 4
        self.KFS2_PREFERRED = kfs2_preferred


    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan_path(self, sty):
        """DFS路径规划 — 返回最优路径 或 None"""

        # 重置状态
        self.path = [[], [], [], [], []]
        
        for y in sty:
            for i in range(5):
                self.visited = np.zeros((self.GRID_ROWS, self.GRID_COLS), dtype=bool)
                self.cnt_path.clear()
                self.eval_path.clear()
                self.kfs2_indicated=i
                self._dfs(0, y, 0)

        return self.path

    # ------------------------------------------------------------------
    # DFS 遍历
    # ------------------------------------------------------------------

    def _dfs(self, posX, posY, yaw):
        if posX == 5:
            self.cnt_path.append([0, float(posX), float(posY), 0.0, self.method*float(yaw), 0.0, 0.0, 0.0])
            self._copy_path(self.cnt_path, self.eval_path)
            self._evaluate_path()
            self.cnt_path.pop()
            return

        self.visited[posX][posY] = True
        self.cnt_path.append([0, float(posX), float(posY), 0.0, self.method*float(yaw), 0.0, 0.0, 0.0])
        
        for direction in self.DIRECTIONS:
            newX = posX + direction[0]
            newY = posY + direction[1]
            if newX < 0 or newX > 5 or newY < 0 or newY > 2:
                continue
            if self.visited[newX][newY] or self.GRID[newX][newY] == 3:
                continue
            self._dfs(newX, newY, direction[2])

        self.visited[posX][posY] = False
        self.cnt_path.pop()

    # ------------------------------------------------------------------
    # 路径评估
    # ------------------------------------------------------------------

    @staticmethod
    def _copy_path(src, dst):
        dst.clear()
        for step in src:
            dst.append(step[:])

    @staticmethod
    def _get_dir_idx(yaw):
        if abs(yaw - 0) < 1e-6:
            return 0
        elif abs(yaw - math.pi / 2) < 1e-6:
            return 1
        elif abs(yaw + math.pi / 2) < 1e-6:
            return 2
        return 0

    def _evaluate_path(self):
        # ---- 1. 检查首排 kfs2 ----
        kfs2_needed_in_first_row = 0
        kfs2_needed = 0
        if 2 in self.GRID[1]:
            kfs2_needed_in_first_row=1

        grid_cnt = self.GRID.copy()

        # ---- 2. 必经之路上的 kfs2 ----
        kfs2_on_the_way = []
        for step in self.eval_path:
            nowX, nowY = round(step[1]), round(step[2])
            if grid_cnt[nowX][nowY] == 2:
                kfs2_on_the_way.append([nowX, nowY])
                grid_cnt[nowX][nowY] = 0

        # ---- 3. 朝向刚好有的 kfs2 ----
        kfs2_by_the_way = []
        for step in self.eval_path:
            now_dir = self._get_dir_idx(step[4])
            nowX, nowY = round(step[1]), round(step[2])
            nx = nowX + self.DIRECTIONS[now_dir][0]
            ny = nowY + self.DIRECTIONS[now_dir][1]
            if 0 <= nx <= 5 and 0 <= ny <= 2:
                if grid_cnt[nx][ny] == 2:
                    kfs2_by_the_way.append([nx, ny])
                    grid_cnt[nx][ny] = 0

        # ---- 4. 通过转向能拿到的 kfs2 ----
        kfs2_can_get = []
        for step in self.eval_path:
            nowX, nowY = round(step[1]), round(step[2])
            for direction in self.DIRECTIONS:
                nx = nowX + direction[0]
                ny = nowY + direction[1]
                if 0 <= nx <= 5 and 0 <= ny <= 2:
                    if grid_cnt[nx][ny] == 2:
                        kfs2_can_get.append([nx, ny])
                        grid_cnt[nx][ny] = 0

        # ---- 5. 必经之路上的 kfs1 ----
        kfs1_on_the_way = []
        for step in self.eval_path:
            nowX, nowY = round(step[1]), round(step[2])
            if grid_cnt[nowX][nowY] == 1:
                kfs1_on_the_way.append([nowX - 1, nowY])
                grid_cnt[nowX][nowY] = 0

        # ---- 6. 计算需要取的 kfs2 数量 ----
        kfs2_by_the_way_needed = 0
        kfs2_can_get_needed = 0
            
        kfs2_by_the_way_needed = max(0, min(
            self.kfs2_indicated - len(kfs2_on_the_way),
            len(kfs2_by_the_way),
        ))
        kfs2_can_get_needed = max(0, min(
            self.kfs2_indicated - len(kfs2_on_the_way) - kfs2_by_the_way_needed,
            len(kfs2_can_get),
        ))
        # Trim excess
        del kfs2_by_the_way[kfs2_by_the_way_needed:]
        del kfs2_can_get[kfs2_can_get_needed:]
        kfs2_needed += len(kfs2_on_the_way) + kfs2_by_the_way_needed + kfs2_can_get_needed


        # ---- 7. 可行性检查 ----
        if kfs2_needed != self.kfs2_indicated:
            return

        # ---- 8. 构建输出路径（纯 grid 索引，无坐标转换） ----
        pub_path = []

        for i, step in enumerate(self.eval_path):
            grid_row = round(step[1])
            grid_col = round(step[2])

            pub_path.append([0, grid_row, grid_col, 0, step[4], 0, 0, 0])

            extra_steps = []
            next_step = self.eval_path[i + 1] if i + 1 < len(self.eval_path) else None

            # 朝向刚好有的 kfs2
            now_dir = self._get_dir_idx(step[4])
            nx = round(step[1]) + self.DIRECTIONS[now_dir][0]
            ny = round(step[2]) + self.DIRECTIONS[now_dir][1]
            if (kfs2_by_the_way and 0 <= nx <= 5 and 0 <= ny <= 2
                    and kfs2_by_the_way[0] == [nx, ny]):
                nr, nc = kfs2_by_the_way.pop(0)
                h = self.kfs_grid_height[nr][nc] - self.kfs_grid_height[round(step[1])][round(step[2])]
                extra_steps.insert(0, [1, nr, nc, h, step[4], 0, 0, 0])

            # 通过转向能拿到的 kfs2
            for direction in self.DIRECTIONS:
                if not kfs2_can_get:
                    break
                nx = round(step[1]) + direction[0]
                ny = round(step[2]) + direction[1]
                if (0 <= nx <= 5 and 0 <= ny <= 2
                        and kfs2_can_get[0] == [nx, ny]):
                    nr, nc = kfs2_can_get.pop(0)
                    h = self.kfs_grid_height[nr][nc] - self.kfs_grid_height[round(step[1])][round(step[2])]
                    extra_steps.append([1, nr, nc, h, direction[2], 0, 0, 0])
            
            # 必经之路上的 kfs2 — 在到达下一步之前抓取
            if (kfs2_on_the_way and next_step is not None
                    and kfs2_on_the_way[0] == [round(next_step[1]), round(next_step[2])]):
                nr, nc = kfs2_on_the_way.pop(0)
                h = self.kfs_grid_height[nr][nc] - self.kfs_grid_height[round(step[1])][round(step[2])]
                extra_steps.append([1, nr, nc, h, next_step[4], 0, 0, 0])

            pub_path.extend(extra_steps)

        # ---- 9. 首排 kfs2 未取则进入 grid 即无效 ----
        if kfs2_needed_in_first_row != 0:
            grabbed = False
            for step in pub_path:
                if step[0] == 0 and step[1] > 0 and not grabbed:
                    return
                if step[0] == 1:
                    grabbed = True

        # ---- 10. 计算代价 ----
        cost = 0
        for i, step in enumerate(pub_path):
            pre = pub_path[i - 1] if i > 0 else None
            if pre is not None and step[4] != pre[4] and self.method==1:
                cost += self.TURN_COST
            if pre is not None and (pre[1] != step[1] or pre[2] != step[2]):
                cost += self.MOVE_COST
            if step[0] == 1:
                cost += self.FETCH_KFS2_COST

        self._add_path(pub_path, cost, kfs2_needed, kfs1_on_the_way)


    # ------------------------------------------------------------------
    # 路径存储与排序
    # ------------------------------------------------------------------

    def _add_path(self, path, cost, kfs2_needed, kfs1_affected):
        """按 bucket 插入，bucket 内按 (cost, kfs1数量) 升序"""
        bucket = self.path[kfs2_needed]
        bucket.append((path, cost, kfs2_needed, kfs1_affected))
        pos = len(bucket) - 1
        i = pos - 1
        while i >= 0 and (
            bucket[i][1] > bucket[pos][1]
            or (bucket[i][1] == bucket[pos][1]
                and len(bucket[i][3]) > len(bucket[pos][3]))
        ):
            i -= 1
        bucket.insert(i + 1, bucket.pop(pos))