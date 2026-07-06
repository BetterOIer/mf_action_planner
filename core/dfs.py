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

路径评估（机械结构调整后）：

代价 = 台阶高度差耗时 + 抓取耗时
  - 平层移动: 1.0s
  - 上+200mm台阶: 2.5s   上+400mm台阶: 4.0s
  - 下-200mm台阶: 3.5s   下-400mm台阶: 5.0s
  - 抓取 KFS2（与上一步同向）: 4.0s
  - 抓取 KFS2（与上一步异向，有转向）: 4.5s

移动模型：
  - 单向模式 (method=1)：无独立转向步。抓取步自带新朝向，后续所有 move 步
    沿用此朝向，直到下一次抓取改变朝向。
  - 全向模式 (method=0)：朝向恒为 0。

第一优先：路径的代价（越低越好）
第二优先：受影响的kfs1数量（越少越好）
第三优先：取到kfs2的数量 2个优先于3个


"""

import math

import numpy as np

# ------------------------------------------------------------------
# 代价常量
# ------------------------------------------------------------------
_FETCH_COST_NO_TURN = 4.0   # 抓取时与上一步同向
_FETCH_COST_TURN = 5.0      # 抓取时与上一步异向（有转向）

# 高度差 → 耗时 (s) 映射
_DH_COST = {
    0:   1.0,   # 平层移动
    200: 2.5,   # 上+200mm
    400: 4.0,   # 上+400mm
    -200: 3.5,  # 下-200mm
    -400: 5.0,  # 下-400mm
}


class DFSPlanner:
    def __init__(self, grid_cols, grid_rows, grid, kfs_grid_height,
                 method=1, logger=None,
                 move_cost=2, turn_cost=1, fetch_kfs2_cost=4):
        """move_cost / turn_cost 保留仅为兼容旧调用，实际不再使用。"""
        self.logger = logger
        self.GRID_ROWS = grid_rows
        self.GRID_COLS = grid_cols
        self.DIRECTIONS = [(1, 0, 0), (0, 1, math.pi / 2), (0, -1, -math.pi / 2)]

        self.method = method  # 1=uni, 0=omni

        self.GRID = grid
        self.kfs_grid_height = kfs_grid_height
        self.visited = np.zeros((self.GRID_ROWS, self.GRID_COLS), dtype=bool)
        self.kfs2_indicated = 0
        self.path = [[], [], [], [], []]
        self.cnt_path = []
        self.eval_path = []

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan_path(self, sty):
        """DFS路径规划 — 返回各 kfs2 数量（0-4）的最优路径列表"""

        self.path = [[], [], [], [], []]

        for y in sty:
            for i in range(5):
                self.visited = np.zeros((self.GRID_ROWS, self.GRID_COLS), dtype=bool)
                self.cnt_path.clear()
                self.eval_path.clear()
                self.kfs2_indicated = i
                self._dfs(0, y, 0)

        return self.path

    # ------------------------------------------------------------------
    # DFS 遍历
    # ------------------------------------------------------------------

    def _dfs(self, posX, posY, yaw):
        if posX == 5:
            h = float(self.kfs_grid_height[posX][posY])
            self.cnt_path.append(
                [0, float(posX), float(posY), h, self.method * float(yaw), 0.0, 0.0, 0.0]
            )
            self._copy_path(self.cnt_path, self.eval_path)
            self._evaluate_path()
            self.cnt_path.pop()
            return

        self.visited[posX][posY] = True
        h = float(self.kfs_grid_height[posX][posY])
        self.cnt_path.append(
            [0, float(posX), float(posY), h, self.method * float(yaw), 0.0, 0.0, 0.0]
        )

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
    def _insert_safety_moves(path):
        """
        硬件安全约束：连续抓取操作不得超过1次。
        当出现连续>=2次fetch时，每两个fetch之间插入一个move，
        将机器人移回最近的真实move位置，并以当前fetch的yaw朝向。"""
        if not path:
            return path

        result = []
        i = 0
        last_move_coords = None  # (x, y, z) 最近的真实 move

        while i < len(path):
            step = path[i]
            if step[0] == 0:  # move
                last_move_coords = (step[1], step[2], step[3])
                result.append(step)
                i += 1
            elif step[0] == 1:  # fetch
                # 收集连续 fetch
                fetch_run = []
                while i < len(path) and path[i][0] == 1:
                    fetch_run.append(path[i])
                    i += 1

                if len(fetch_run) >= 2:
                    for j, fstep in enumerate(fetch_run):
                        result.append(fstep)
                        # 每个 fetch 之后（除了最后一个）插入一个 move
                        if j != len(fetch_run) - 1:
                            if last_move_coords is not None:
                                mx, my, mz = last_move_coords
                            else:
                                mx, my, mz = fstep[1], fstep[2], 0.0
                            # yaw 取自当前 fetch（插入位置前面的那个）
                            myaw = fstep[4]
                            result.append([0, mx, my, mz, myaw, 0.0, 0.0, 0.0])
                else:
                    result.extend(fetch_run)

        return result

    @staticmethod
    def _get_dir_idx(yaw):
        if abs(yaw - 0) < 1e-6:
            return 0
        elif abs(yaw - math.pi / 2) < 1e-6:
            return 1
        elif abs(yaw + math.pi / 2) < 1e-6:
            return 2
        return 0

    @staticmethod
    def _has_backward_step_with_height_change(path):
        """检查路径中是否存在运动方向与朝向完全相反且高度有变化的步。

        只比较连续的两个移动步（type == 0），跳过中间的抓取步，
        因为抓取步的位置是 KFS 坐标而非机器人坐标，直接比较会误判方向。

        如果存在这样的步（移动方向与朝向完全相反 且 高度有变化），
        返回 True，表示应放弃该路径。
        """
        last_move = None
        for step in path:
            if step[0] != 0:          # 跳过抓取步 (type == 1)
                continue

            if last_move is not None:
                dx = step[1] - last_move[1]   # x 方向位移
                dy = step[2] - last_move[2]   # y 方向位移
                dh = step[3] - last_move[3]   # 高度差

                if abs(dh) >= 1e-6:            # 有高度变化才检查
                    yaw = step[4]               # 当前步的朝向

                    # 朝向 0 (面向 +x) 但向 -x 移动
                    if abs(yaw - 0) < 1e-6 and dx < -1e-6:
                        return True
                    # 朝向 pi/2 (面向 +y) 但向 -y 移动
                    if abs(yaw - math.pi / 2) < 1e-6 and dy < -1e-6:
                        return True
                    # 朝向 -pi/2 (面向 -y) 但向 +y 移动
                    if abs(yaw + math.pi / 2) < 1e-6 and dy > 1e-6:
                        return True

            last_move = step

        return False

    def _evaluate_path(self):
        # ---- 1. 检查首排 kfs2 ----
        kfs2_needed_in_first_row = 0
        kfs2_needed = 0
        if 2 in self.GRID[1]:
            kfs2_needed_in_first_row = 1

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

        # ---- 8. 构建输出路径 ----
        pub_path = []
        # 单向模式：跟踪上次抓取后的朝向，后续 move 步沿用此朝向直到下次抓取
        effective_yaw = 0.0

        for i, step in enumerate(self.eval_path):
            grid_row = round(step[1])
            grid_col = round(step[2])

            move_h = float(self.kfs_grid_height[grid_row][grid_col])
            if self.method == 0:
                move_yaw = 0.0
            else:
                move_yaw = effective_yaw

            pub_path.append([0, grid_row, grid_col, move_h, move_yaw, 0, 0, 0])

            extra_steps = []
            next_step = self.eval_path[i + 1] if i + 1 < len(self.eval_path) else None

            # ---- 朝向刚好有的 kfs2（方向与当前朝向一致） ----
            now_dir = self._get_dir_idx(step[4])
            nx = round(step[1]) + self.DIRECTIONS[now_dir][0]
            ny = round(step[2]) + self.DIRECTIONS[now_dir][1]
            if (kfs2_by_the_way and 0 <= nx <= 5 and 0 <= ny <= 2
                    and kfs2_by_the_way[0] == [nx, ny]):
                nr, nc = kfs2_by_the_way.pop(0)
                h = self.kfs_grid_height[nr][nc] - self.kfs_grid_height[round(step[1])][round(step[2])]
                fetch_yaw = 0.0 if self.method == 0 else step[4]
                extra_steps.insert(0, [1, nr, nc, h, fetch_yaw, 0, 0, 0])
                if self.method == 1:
                    effective_yaw = step[4]

            # ---- 通过转向能拿到的 kfs2 ----
            for direction in self.DIRECTIONS:
                if not kfs2_can_get:
                    break
                nx = round(step[1]) + direction[0]
                ny = round(step[2]) + direction[1]
                if (0 <= nx <= 5 and 0 <= ny <= 2
                        and kfs2_can_get[0] == [nx, ny]):
                    nr, nc = kfs2_can_get.pop(0)
                    h = self.kfs_grid_height[nr][nc] - self.kfs_grid_height[round(step[1])][round(step[2])]
                    # 抓取步自带新朝向，不另加转向步
                    fetch_yaw = 0.0 if self.method == 0 else direction[2]
                    extra_steps.append([1, nr, nc, h, fetch_yaw, 0, 0, 0])
                    if self.method == 1:
                        effective_yaw = direction[2]

            # ---- 必经之路上的 kfs2 — 在到达下一步之前抓取 ----
            if (kfs2_on_the_way and next_step is not None
                    and kfs2_on_the_way[0] == [round(next_step[1]), round(next_step[2])]):
                nr, nc = kfs2_on_the_way.pop(0)
                h = self.kfs_grid_height[nr][nc] - self.kfs_grid_height[round(step[1])][round(step[2])]
                fetch_yaw = 0.0 if self.method == 0 else next_step[4]
                extra_steps.append([1, nr, nc, h, fetch_yaw, 0, 0, 0])
                if self.method == 1:
                    effective_yaw = next_step[4]

            pub_path.extend(extra_steps)

        # ---- 8.5 硬件安全：连续抓取不超过2次 ----
        pub_path = self._insert_safety_moves(pub_path)

        # ---- 8.6 检查运动方向与朝向相反且高度变化（上/下台阶） ----
        if self._has_backward_step_with_height_change(pub_path):
            return

        # ---- 9. 首排 kfs2 未取则进入 grid 即无效 ----
        if kfs2_needed_in_first_row != 0:
            grabbed = False
            for step in pub_path:
                if step[0] == 0 and step[1] > 0 and not grabbed:
                    return
                if step[0] == 1:
                    grabbed = True

        # ---- 10. 计算代价（高度差 + 抓取） ----
        cost = 0.0
        last_x, last_y, last_h = 0.0, float(round(self.eval_path[0][2])), 0.0
        last_yaw = 0.0  # 初始朝向
        for step in pub_path:
            if step[0] == 1:
                # 抓取步：与上一步方向比较，异向则加转向耗时
                if abs(step[4] - last_yaw) > 1e-6:
                    cost += _FETCH_COST_TURN
                else:
                    cost += _FETCH_COST_NO_TURN
                last_yaw = step[4]
            elif step[0] == 0:
                sx, sy, sh = step[1], step[2], step[3]
                if abs(sx - last_x) > 1e-6 or abs(sy - last_y) > 1e-6:
                    dh = int(round(sh - last_h))
                    cost += _DH_COST.get(dh, 1.0)
                    last_x, last_y, last_h = sx, sy, sh
                last_yaw = step[4]

        self._add_path(pub_path, cost, kfs2_needed, kfs1_on_the_way)

    # ------------------------------------------------------------------
    # 路径存储与排序
    # ------------------------------------------------------------------

    @staticmethod
    def _count_turns(path):
        """统计路径中朝向变化的次数"""
        turns = 0
        for i in range(1, len(path)):
            if abs(path[i][4] - path[i - 1][4]) > 1e-6:
                turns += 1
        return turns

    def _add_path(self, path, cost, kfs2_needed, kfs1_affected):
        """按 bucket 插入，bucket 内按 (cost, kfs1数量, 转向次数) 升序"""
        bucket = self.path[kfs2_needed]
        turns = self._count_turns(path)
        bucket.append((path, cost, kfs2_needed, kfs1_affected, turns))
        pos = len(bucket) - 1
        i = pos - 1
        while i >= 0 and (
            bucket[i][1] > bucket[pos][1]
            or (bucket[i][1] == bucket[pos][1]
                and len(bucket[i][3]) > len(bucket[pos][3]))
            or (bucket[i][1] == bucket[pos][1]
                and len(bucket[i][3]) == len(bucket[pos][3])
                and bucket[i][4] > bucket[pos][4])
        ):
            i -= 1
        bucket.insert(i + 1, bucket.pop(pos))
