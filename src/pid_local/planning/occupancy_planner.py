import heapq
import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np


Point2 = Tuple[float, float]
WallSeg = Tuple[float, float, float, float]


class VisibilityPlanner:
    """Occupancy-grid planner with the legacy VisibilityPlanner API."""

    STRICT_EDGE_CLEARANCE_MM = 1.0

    def __init__(
        self,
        walls_mm: Sequence[WallSeg],
        inflation_mm: float,
        helper_edge_length_mm: float = 40.0,
        center_lines_mm: Sequence[WallSeg] | None = None,
    ):
        self.walls_mm = list(walls_mm)
        self.inflation_mm = float(inflation_mm)
        self.helper_edge_length_mm = float(helper_edge_length_mm)
        self.center_lines_mm = list(center_lines_mm or [])

        if not self.walls_mm:
            raise ValueError("walls_mm must not be empty")

        xs = [x for w in self.walls_mm for x in (float(w[0]), float(w[2]))]
        ys = [y for w in self.walls_mm for y in (float(w[1]), float(w[3]))]
        self.min_x = min(xs)
        self.max_x = max(xs)
        self.min_y = min(ys)
        self.max_y = max(ys)

        # Keep resolution linked to existing helper spacing knob.
        self.grid_resolution_mm = max(2.0, min(8.0, self.helper_edge_length_mm / 8.0))
        self.effective_inflation_mm = self.inflation_mm + float(self.STRICT_EDGE_CLEARANCE_MM)

        self.x_coords = np.arange(self.min_x, self.max_x + self.grid_resolution_mm, self.grid_resolution_mm)
        self.y_coords = np.arange(self.min_y, self.max_y + self.grid_resolution_mm, self.grid_resolution_mm)
        self.width = int(len(self.x_coords))
        self.height = int(len(self.y_coords))
        self.occupancy = np.zeros((self.height, self.width), dtype=bool)

        self._build_occupancy()

    @staticmethod
    def _point_to_segment_dist(px, py, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        if dx == dy == 0:
            return math.hypot(px - x1, py - y1)

        denom = (dx * dx + dy * dy)
        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / denom))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return math.hypot(px - proj_x, py - proj_y)

    def _point_blocked(self, x_mm: float, y_mm: float) -> bool:
        # Keep center-point workspace inset from outer bounds.
        if x_mm < (self.min_x + self.effective_inflation_mm) or x_mm > (self.max_x - self.effective_inflation_mm):
            return True
        if y_mm < (self.min_y + self.effective_inflation_mm) or y_mm > (self.max_y - self.effective_inflation_mm):
            return True

        for x1, y1, x2, y2 in self.walls_mm:
            if self._point_to_segment_dist(
                x_mm,
                y_mm,
                float(x1),
                float(y1),
                float(x2),
                float(y2),
            ) <= self.effective_inflation_mm:
                return True
        return False

    def _build_occupancy(self):
        for iy, y_mm in enumerate(self.y_coords):
            for ix, x_mm in enumerate(self.x_coords):
                self.occupancy[iy, ix] = self._point_blocked(float(x_mm), float(y_mm))

    def _in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.width and 0 <= iy < self.height

    def world_to_cell(self, point_xy: Iterable[float]) -> tuple[int, int]:
        x_mm, y_mm = tuple(point_xy)
        ix = int(round((float(x_mm) - self.min_x) / self.grid_resolution_mm))
        iy = int(round((float(y_mm) - self.min_y) / self.grid_resolution_mm))
        ix = max(0, min(self.width - 1, ix))
        iy = max(0, min(self.height - 1, iy))
        return ix, iy

    def cell_to_world(self, ix: int, iy: int) -> Point2:
        return float(self.x_coords[ix]), float(self.y_coords[iy])

    def nearest_free_cell(self, ix: int, iy: int, max_radius: int = 48):
        if self._in_bounds(ix, iy) and not self.occupancy[iy, ix]:
            return ix, iy

        for radius in range(1, max_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in (-radius, radius):
                    nx, ny = ix + dx, iy + dy
                    if self._in_bounds(nx, ny) and not self.occupancy[ny, nx]:
                        return nx, ny
            for dy in range(-radius + 1, radius):
                for dx in (-radius, radius):
                    nx, ny = ix + dx, iy + dy
                    if self._in_bounds(nx, ny) and not self.occupancy[ny, nx]:
                        return nx, ny
        return None

    def nearest_free_world(self, point_xy: Iterable[float], max_radius: int = 48):
        ix, iy = self.world_to_cell(point_xy)
        free_cell = self.nearest_free_cell(ix, iy, max_radius=max_radius)
        if free_cell is None:
            return None
        return self.cell_to_world(free_cell[0], free_cell[1])

    def plan(self, start_xy: Iterable[float], goal_xy: Iterable[float]) -> List[Point2]:
        start_xy = tuple(float(v) for v in start_xy)
        goal_xy = tuple(float(v) for v in goal_xy)

        sx, sy = self.world_to_cell(start_xy)
        gx, gy = self.world_to_cell(goal_xy)

        start_cell = self.nearest_free_cell(sx, sy)
        goal_cell = self.nearest_free_cell(gx, gy)
        if start_cell is None or goal_cell is None:
            return [start_xy]

        if start_cell == goal_cell:
            return [start_xy, goal_xy]

        neighbors = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, 1.4142),
            (-1, 1, 1.4142),
            (1, -1, 1.4142),
            (1, 1, 1.4142),
        ]

        def heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
            return float(math.hypot(a[0] - b[0], a[1] - b[1]))

        open_heap: list[tuple[float, tuple[int, int]]] = []
        heapq.heappush(open_heap, (0.0, start_cell))
        g_score = {start_cell: 0.0}
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        closed = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == goal_cell:
                break
            closed.add(current)

            cx, cy = current
            for dx, dy, step_cost in neighbors:
                nx, ny = cx + dx, cy + dy
                if not self._in_bounds(nx, ny) or self.occupancy[ny, nx]:
                    continue

                # Prevent diagonal corner cutting through occupied cells.
                if dx != 0 and dy != 0:
                    if self.occupancy[cy, nx] or self.occupancy[ny, cx]:
                        continue

                nxt = (nx, ny)
                tentative = g_score[current] + step_cost
                if tentative < g_score.get(nxt, float("inf")):
                    g_score[nxt] = tentative
                    came_from[nxt] = current
                    f_score = tentative + heuristic(nxt, goal_cell)
                    heapq.heappush(open_heap, (f_score, nxt))

        if goal_cell not in came_from:
            return [start_xy]

        path_cells = [goal_cell]
        cur = goal_cell
        while cur != start_cell:
            cur = came_from[cur]
            path_cells.append(cur)
        path_cells.reverse()

        path_mm = [self.cell_to_world(ix, iy) for ix, iy in path_cells]
        path_mm[0] = start_xy
        path_mm[-1] = goal_xy
        return [(float(x), float(y)) for x, y in path_mm]
