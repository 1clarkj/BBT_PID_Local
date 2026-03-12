import math
from typing import Iterable, List, Sequence, Tuple

import networkx as nx
import numpy as np
from shapely.geometry import LineString, Point, box
from shapely.ops import unary_union


Point2 = Tuple[float, float]
WallSeg = Tuple[float, float, float, float]


class VisibilityPlanner:
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

        raw_bounds_polygon, self.internal_walls = self._extract_bounds_and_internal_walls(self.walls_mm)
        # Plan in the center-point workspace (inset from outer walls by inflation).
        inset_bounds = raw_bounds_polygon.buffer(-self.inflation_mm, join_style=2)
        if inset_bounds.is_empty:
            self.bounds_polygon = raw_bounds_polygon
        else:
            self.bounds_polygon = inset_bounds
        self.obstacles = self._build_obstacles(self.internal_walls, self.inflation_mm)
        self.base_nodes = self._build_nodes(
            self.center_lines_mm,
            self.obstacles,
            self.bounds_polygon,
            self.helper_edge_length_mm,
        )
        self.base_graph = self._build_visibility_graph(
            self.base_nodes,
            self.center_lines_mm,
            self.obstacles,
            self.bounds_polygon,
            self.helper_edge_length_mm,
        )

    @staticmethod
    def _extract_bounds_and_internal_walls(walls_mm: Sequence[WallSeg]):
        xs = [x for wall in walls_mm for x in (wall[0], wall[2])]
        ys = [y for wall in walls_mm for y in (wall[1], wall[3])]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        outer_walls = []
        internal_walls = []
        for wall in walls_mm:
            x1, y1, x2, y2 = wall
            if (
                (x1 == x2 and x1 in (min_x, max_x))
                or (y1 == y2 and y1 in (min_y, max_y))
            ):
                outer_walls.append(wall)
            else:
                internal_walls.append(wall)

        bounds_polygon = box(min_x, min_y, max_x, max_y)
        return bounds_polygon, internal_walls

    @staticmethod
    def _build_obstacles(walls_mm: Sequence[WallSeg], inflation_mm: float):
        if not walls_mm:
            return []
        segments = [LineString([(x1, y1), (x2, y2)]) for x1, y1, x2, y2 in walls_mm]

        # Add short centerline extensions at dangling endpoints so wall tips
        # get a boundary area even with flat buffer caps.
        endpoint_degree = {}
        endpoint_dir = {}
        for x1, y1, x2, y2 in walls_mm:
            a = (float(x1), float(y1))
            b = (float(x2), float(y2))

            endpoint_degree[a] = endpoint_degree.get(a, 0) + 1
            endpoint_degree[b] = endpoint_degree.get(b, 0) + 1

            ab = np.array([b[0] - a[0], b[1] - a[1]], dtype=float)
            ba = np.array([a[0] - b[0], a[1] - b[1]], dtype=float)
            norm_ab = float(np.linalg.norm(ab))
            norm_ba = float(np.linalg.norm(ba))
            if norm_ab > 1e-9:
                endpoint_dir[b] = ab / norm_ab
            if norm_ba > 1e-9:
                endpoint_dir[a] = ba / norm_ba

        tip_extensions = []
        tip_extension_mm = 1.0 * float(inflation_mm)
        for endpoint, degree in endpoint_degree.items():
            if degree != 1:
                continue
            direction = endpoint_dir.get(endpoint)
            if direction is None:
                continue
            ex = endpoint[0] + float(direction[0]) * tip_extension_mm
            ey = endpoint[1] + float(direction[1]) * tip_extension_mm
            tip_extensions.append(LineString([endpoint, (ex, ey)]))

        merged_centerlines = unary_union([*segments, *tip_extensions])
        merged = merged_centerlines.buffer(
            inflation_mm,
            cap_style=2,
            join_style=2,
            quad_segs=6,
        )
        if merged.geom_type == "Polygon":
            return [merged]
        return list(merged.geoms)

    @staticmethod
    def _segment_sample_points(line_seg: WallSeg, spacing_mm: float):
        x1, y1, x2, y2 = map(float, line_seg)
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        if length == 0:
            return [(round(x1, 3), round(y1, 3))]

        steps = max(int(math.ceil(length / spacing_mm)), 1)
        points = []
        for idx in range(steps + 1):
            t = idx / steps
            px = x1 + t * dx
            py = y1 + t * dy
            points.append((round(px, 3), round(py, 3)))
        return points

    @staticmethod
    def _line_intersection(a_seg: WallSeg, b_seg: WallSeg):
        line_a = LineString([(float(a_seg[0]), float(a_seg[1])), (float(a_seg[2]), float(a_seg[3]))])
        line_b = LineString([(float(b_seg[0]), float(b_seg[1])), (float(b_seg[2]), float(b_seg[3]))])
        inter = line_a.intersection(line_b)
        if inter.is_empty or inter.geom_type != "Point":
            return None
        return (round(float(inter.x), 3), round(float(inter.y), 3))

    @staticmethod
    def _line_direction(line_seg: WallSeg):
        x1, y1, x2, y2 = map(float, line_seg)
        dx = x2 - x1
        dy = y2 - y1
        norm = math.hypot(dx, dy)
        if norm <= 1e-9:
            return None
        return (round(dx / norm, 6), round(dy / norm, 6))

    @classmethod
    def _sample_centerline_nodes(
        cls,
        center_lines_mm: Sequence[WallSeg],
        obstacles,
        bounds_polygon,
        spacing_mm: float,
    ):
        node_set = set()
        endpoint_to_dirs = {}

        for line_seg in center_lines_mm:
            x1, y1, x2, y2 = map(float, line_seg)
            a = (round(x1, 3), round(y1, 3))
            b = (round(x2, 3), round(y2, 3))
            node_set.add(a)
            node_set.add(b)

            direction = cls._line_direction(line_seg)
            if direction is not None:
                endpoint_to_dirs.setdefault(a, []).append(direction)
                endpoint_to_dirs.setdefault(b, []).append((-direction[0], -direction[1]))

            for point in cls._segment_sample_points(line_seg, spacing_mm):
                node_set.add(point)

        for i in range(len(center_lines_mm)):
            for j in range(i + 1, len(center_lines_mm)):
                inter = cls._line_intersection(center_lines_mm[i], center_lines_mm[j])
                if inter is None:
                    continue
                node_set.add(inter)

        for endpoint, dirs in endpoint_to_dirs.items():
            if len(dirs) < 2:
                continue
            has_corner = False
            for i in range(len(dirs)):
                for j in range(i + 1, len(dirs)):
                    dot = dirs[i][0] * dirs[j][0] + dirs[i][1] * dirs[j][1]
                    if abs(dot) < 0.999:
                        has_corner = True
                        break
                if has_corner:
                    break
            if has_corner:
                node_set.add(endpoint)

        free_nodes = set()
        for point in node_set:
            point_xy = np.array(point, dtype=float)
            if cls._point_has_clearance(
                point_xy,
                obstacles,
                bounds_polygon,
                cls.STRICT_EDGE_CLEARANCE_MM,
            ):
                free_nodes.add((round(float(point_xy[0]), 3), round(float(point_xy[1]), 3)))
        return free_nodes

    @classmethod
    def _sample_boundary_points(cls, obstacles, spacing_mm: float):
        point_set = set()
        for poly in obstacles:
            coords = list(poly.exterior.coords)
            for x_mm, y_mm in coords[:-1]:
                point_set.add((round(float(x_mm), 3), round(float(y_mm), 3)))

            for idx in range(len(coords) - 1):
                x1, y1 = coords[idx]
                x2, y2 = coords[idx + 1]
                point_set.update(cls._segment_sample_points((x1, y1, x2, y2), spacing_mm))
        return point_set

    @classmethod
    def _build_nodes(
        cls,
        center_lines_mm: Sequence[WallSeg],
        obstacles,
        bounds_polygon,
        helper_edge_length_mm: float,
    ) -> List[np.ndarray]:
        spacing_mm = max(float(helper_edge_length_mm) * 0.6, 14.0)
        min_x, min_y, max_x, max_y = bounds_polygon.bounds
        node_set = set()

        centerline_spacing = max(float(helper_edge_length_mm) * 0.6, 14.0)
        node_set.update(
            cls._sample_centerline_nodes(
                center_lines_mm,
                obstacles,
                bounds_polygon,
                centerline_spacing,
            )
        )

        # Add a thin lateral band around centerlines to improve doorway/turn
        # transitions without returning to a high global density.
        offset_mm = min(0.35 * spacing_mm, 10.0)
        for line_seg in center_lines_mm:
            x1, y1, x2, y2 = map(float, line_seg)
            dx = x2 - x1
            dy = y2 - y1
            norm = math.hypot(dx, dy)
            if norm <= 1e-9:
                continue
            nx_hat = -dy / norm
            ny_hat = dx / norm
            for point in cls._segment_sample_points(line_seg, centerline_spacing):
                p = np.array([float(point[0]), float(point[1])], dtype=float)
                for sign in (-1.0, 1.0):
                    q = p + sign * offset_mm * np.array([nx_hat, ny_hat], dtype=float)
                    if cls._point_has_clearance(
                        q,
                        obstacles,
                        bounds_polygon,
                        cls.STRICT_EDGE_CLEARANCE_MM,
                    ):
                        node_set.add((round(float(q[0]), 3), round(float(q[1]), 3)))

        # Add a moderate global grid for robust component connectivity without
        # the heavy density of the previous fine sampler.
        y_values = np.arange(min_y + 0.5 * spacing_mm, max_y, spacing_mm)
        grid_seed_nodes = []
        for row_idx, y in enumerate(y_values):
            x_offset = 0.0 if row_idx % 2 == 0 else 0.5 * spacing_mm
            x_start = min_x + 0.5 * spacing_mm + x_offset
            x_values = np.arange(x_start, max_x, spacing_mm)
            for x in x_values:
                point = np.array([float(x), float(y)], dtype=float)
                if cls._point_has_clearance(
                    point,
                    obstacles,
                    bounds_polygon,
                    cls.STRICT_EDGE_CLEARANCE_MM,
                ):
                    key = (round(float(x), 3), round(float(y), 3))
                    node_set.add(key)
                    grid_seed_nodes.append(key)

        # Add random free-space samples to improve local alternatives around the centerline skeleton.
        width = max_x - min_x
        height = max_y - min_y
        target_random = int((width * height) / max(spacing_mm * spacing_mm, 1.0) * 0.4)
        rng = np.random.default_rng(0)
        attempts = max(target_random * 10, 900)
        for _ in range(attempts):
            if len(node_set) >= target_random + max(len(center_lines_mm) * 8, 80):
                break
            x = float(rng.uniform(min_x, max_x))
            y = float(rng.uniform(min_y, max_y))
            point = np.array([x, y], dtype=float)
            if cls._point_has_clearance(
                point,
                obstacles,
                bounds_polygon,
                cls.STRICT_EDGE_CLEARANCE_MM,
            ):
                node_set.add((round(x, 3), round(y, 3)))

        local_offsets = (
            np.array([0.4 * spacing_mm, 0.0], dtype=float),
            np.array([-0.4 * spacing_mm, 0.0], dtype=float),
            np.array([0.0, 0.4 * spacing_mm], dtype=float),
            np.array([0.0, -0.4 * spacing_mm], dtype=float),
        )
        for sx, sy in grid_seed_nodes:
            seed = np.array([sx, sy], dtype=float)
            for offset in local_offsets:
                q = seed + offset
                if cls._point_has_clearance(
                    q,
                    obstacles,
                    bounds_polygon,
                    cls.STRICT_EDGE_CLEARANCE_MM,
                ):
                    node_set.add((round(float(q[0]), 3), round(float(q[1]), 3)))

        return [np.array((x, y), dtype=float) for x, y in sorted(node_set)]

    @staticmethod
    def _point_on_segment(point_xy, line_seg: WallSeg, tol: float = 1e-3):
        px, py = float(point_xy[0]), float(point_xy[1])
        x1, y1, x2, y2 = map(float, line_seg)
        cross = abs((px - x1) * (y2 - y1) - (py - y1) * (x2 - x1))
        if cross > tol:
            return False
        return (
            min(x1, x2) - tol <= px <= max(x1, x2) + tol
            and min(y1, y2) - tol <= py <= max(y1, y2) + tol
        )

    @staticmethod
    def _segment_param(point_xy, line_seg: WallSeg):
        px, py = float(point_xy[0]), float(point_xy[1])
        x1, y1, x2, y2 = map(float, line_seg)
        denom = max((x2 - x1) ** 2 + (y2 - y1) ** 2, 1e-6)
        return ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / denom

    @staticmethod
    def _point_is_free(point_xy: np.ndarray, obstacles, bounds_polygon) -> bool:
        pt = Point(float(point_xy[0]), float(point_xy[1]))
        if not bounds_polygon.covers(pt):
            return False
        return not any(poly.contains(pt) for poly in obstacles)

    @staticmethod
    def _segment_is_free(a_xy: np.ndarray, b_xy: np.ndarray, obstacles, bounds_polygon) -> bool:
        line = LineString([(float(a_xy[0]), float(a_xy[1])), (float(b_xy[0]), float(b_xy[1]))])
        if not bounds_polygon.covers(line):
            return False
        for poly in obstacles:
            if line.crosses(poly) or line.within(poly) or poly.contains(line):
                return False
        return True

    @staticmethod
    def _point_has_clearance(
        point_xy: np.ndarray,
        obstacles,
        bounds_polygon,
        clearance_mm: float,
    ) -> bool:
        pt = Point(float(point_xy[0]), float(point_xy[1]))
        swept = pt.buffer(float(clearance_mm))
        if not bounds_polygon.covers(swept):
            return False
        for poly in obstacles:
            if swept.intersects(poly):
                return False
        return True

    @staticmethod
    def _segment_has_clearance(
        a_xy: np.ndarray,
        b_xy: np.ndarray,
        obstacles,
        bounds_polygon,
        clearance_mm: float,
    ) -> bool:
        line = LineString([(float(a_xy[0]), float(a_xy[1])), (float(b_xy[0]), float(b_xy[1]))])
        swept = line.buffer(float(clearance_mm), cap_style=2, join_style=2)
        if not bounds_polygon.covers(swept):
            return False
        for poly in obstacles:
            if swept.intersects(poly):
                return False
        return True

    @classmethod
    def _nearest_free_point(cls, point_xy: np.ndarray, obstacles, bounds_polygon) -> np.ndarray:
        if cls._point_has_clearance(
            point_xy,
            obstacles,
            bounds_polygon,
            cls.STRICT_EDGE_CLEARANCE_MM,
        ):
            return point_xy

        px, py = float(point_xy[0]), float(point_xy[1])
        angles = np.linspace(0.0, 2.0 * math.pi, 32, endpoint=False)
        for r in np.arange(2.0, 100.0, 2.0):
            for theta in angles:
                q = np.array([px + r * math.cos(theta), py + r * math.sin(theta)], dtype=float)
                if cls._point_has_clearance(
                    q,
                    obstacles,
                    bounds_polygon,
                    cls.STRICT_EDGE_CLEARANCE_MM,
                ):
                    return q
        return point_xy

    @classmethod
    def _build_visibility_graph(
        cls,
        nodes: Sequence[np.ndarray],
        center_lines_mm: Sequence[WallSeg],
        obstacles,
        bounds_polygon,
        helper_edge_length_mm: float,
    ):
        graph = nx.Graph()
        for i, node in enumerate(nodes):
            graph.add_node(i, pos=node)

        node_index = {(round(float(node[0]), 3), round(float(node[1]), 3)): idx for idx, node in enumerate(nodes)}
        centerline_spacing = max(float(helper_edge_length_mm) * 0.6, 14.0)
        for line_seg in center_lines_mm:
            line_points = []
            for point in cls._segment_sample_points(line_seg, centerline_spacing):
                p = (round(float(point[0]), 3), round(float(point[1]), 3))
                idx = node_index.get(p)
                if idx is not None:
                    line_points.append((cls._segment_param(nodes[idx], line_seg), idx))
            if not line_points:
                continue
            line_points.sort(key=lambda item: item[0])
            for (_, ia), (_, ib) in zip(line_points[:-1], line_points[1:]):
                a = graph.nodes[ia]["pos"]
                b = graph.nodes[ib]["pos"]
                dist = float(np.linalg.norm(a - b))
                if cls._segment_has_clearance(
                    a,
                    b,
                    obstacles,
                    bounds_polygon,
                    cls.STRICT_EDGE_CLEARANCE_MM,
                ):
                    graph.add_edge(ia, ib, weight=dist)

        max_bridge_distance_mm = max(float(helper_edge_length_mm) * 4.0, 180.0)
        max_neighbors_per_node = 16
        for i, a in enumerate(nodes):
            neighbor_order = sorted(
                ((float(np.linalg.norm(a - b)), j) for j, b in enumerate(nodes) if j != i),
                key=lambda item: item[0],
            )
            added = 0
            for dist, j in neighbor_order:
                if dist > max_bridge_distance_mm and added >= 4:
                    break
                b = nodes[j]
                if not cls._segment_has_clearance(
                    a,
                    b,
                    obstacles,
                    bounds_polygon,
                    cls.STRICT_EDGE_CLEARANCE_MM,
                ):
                    continue
                graph.add_edge(i, j, weight=dist)
                added += 1
                if added >= max_neighbors_per_node:
                    break

        # If local neighbor linking leaves disconnected islands, connect components
        # using only globally shortest clearance-valid bridges.
        while nx.number_connected_components(graph) > 1:
            components = [list(comp) for comp in nx.connected_components(graph)]
            best = None
            for ci in range(len(components)):
                for cj in range(ci + 1, len(components)):
                    comp_a = components[ci]
                    comp_b = components[cj]
                    for ia in comp_a:
                        a = graph.nodes[ia]["pos"]
                        for ib in comp_b:
                            b = graph.nodes[ib]["pos"]
                            dist = float(np.linalg.norm(a - b))
                            if best is not None and dist >= best[0]:
                                continue
                            if not cls._segment_has_clearance(
                                a,
                                b,
                                obstacles,
                                bounds_polygon,
                                cls.STRICT_EDGE_CLEARANCE_MM,
                            ):
                                continue
                            best = (dist, ia, ib)
            if best is None:
                break
            graph.add_edge(best[1], best[2], weight=best[0])
        return graph

    def _nearest_graph_node_with_clearance(self, point_xy: np.ndarray):
        if not self.base_nodes:
            return None

        ordered = sorted(
            ((float(np.linalg.norm(point_xy - node)), idx) for idx, node in enumerate(self.base_nodes)),
            key=lambda item: item[0],
        )
        for _, idx in ordered:
            node = self.base_graph.nodes[idx]["pos"]
            if self._segment_has_clearance(
                point_xy,
                node,
                self.obstacles,
                self.bounds_polygon,
                self.STRICT_EDGE_CLEARANCE_MM,
            ):
                return idx
        return None

    def plan(self, start_xy: Iterable[float], goal_xy: Iterable[float]) -> List[Point2]:
        start = self._nearest_free_point(np.array(tuple(start_xy), dtype=float), self.obstacles, self.bounds_polygon)
        goal = self._nearest_free_point(np.array(tuple(goal_xy), dtype=float), self.obstacles, self.bounds_polygon)

        if self._segment_has_clearance(
            start,
            goal,
            self.obstacles,
            self.bounds_polygon,
            self.STRICT_EDGE_CLEARANCE_MM,
        ):
            return [(float(start[0]), float(start[1])), (float(goal[0]), float(goal[1]))]

        sid = self._nearest_graph_node_with_clearance(start)
        gid = self._nearest_graph_node_with_clearance(goal)
        if sid is None or gid is None:
            return [(float(start[0]), float(start[1]))]

        try:
            index_path = nx.astar_path(
                self.base_graph,
                sid,
                gid,
                heuristic=lambda a, b: float(
                    np.linalg.norm(self.base_graph.nodes[a]["pos"] - self.base_graph.nodes[b]["pos"])
                ),
                weight="weight",
            )
        except nx.NetworkXNoPath:
            return [(float(start[0]), float(start[1]))]

        anchor_start = self.base_graph.nodes[sid]["pos"]
        anchor_goal = self.base_graph.nodes[gid]["pos"]
        if not self._segment_has_clearance(
            start,
            anchor_start,
            self.obstacles,
            self.bounds_polygon,
            self.STRICT_EDGE_CLEARANCE_MM,
        ):
            return [(float(start[0]), float(start[1]))]
        if not self._segment_has_clearance(
            anchor_goal,
            goal,
            self.obstacles,
            self.bounds_polygon,
            self.STRICT_EDGE_CLEARANCE_MM,
        ):
            return [(float(start[0]), float(start[1]))]

        path = [(float(start[0]), float(start[1]))]
        for idx in index_path:
            node = self.base_graph.nodes[idx]["pos"]
            path.append((float(node[0]), float(node[1])))
        path.append((float(goal[0]), float(goal[1])))
        return path
