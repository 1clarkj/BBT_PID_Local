import math
from typing import Iterable, List, Sequence, Tuple

import networkx as nx
import numpy as np
from shapely.geometry import LineString, Point
from shapely.ops import unary_union


Point2 = Tuple[float, float]
WallSeg = Tuple[float, float, float, float]


class VisibilityPlanner:
    def __init__(
        self,
        walls_mm: Sequence[WallSeg],
        inflation_mm: float,
        helper_edge_length_mm: float = 40.0,
    ):
        self.walls_mm = list(walls_mm)
        self.inflation_mm = float(inflation_mm)
        self.helper_edge_length_mm = float(helper_edge_length_mm)

        self.obstacles = self._build_obstacles(self.walls_mm, self.inflation_mm)
        self.base_nodes = self._build_nodes(self.obstacles, self.helper_edge_length_mm)
        self.base_graph = self._build_visibility_graph(self.base_nodes, self.obstacles)

    @staticmethod
    def _build_obstacles(walls_mm: Sequence[WallSeg], inflation_mm: float):
        segments = [LineString([(x1, y1), (x2, y2)]) for x1, y1, x2, y2 in walls_mm]
        inflated = [seg.buffer(inflation_mm, cap_style=2, join_style=2) for seg in segments]
        merged = unary_union(inflated)
        if merged.geom_type == "Polygon":
            return [merged]
        return list(merged.geoms)

    @staticmethod
    def _build_nodes(obstacles, helper_edge_length_mm: float) -> List[np.ndarray]:
        node_set = set()
        for poly in obstacles:
            coords = list(poly.exterior.coords)
            for x, y in coords[:-1]:
                node_set.add((round(x, 3), round(y, 3)))

            for i in range(len(coords) - 1):
                x1, y1 = coords[i]
                x2, y2 = coords[i + 1]
                edge_len = math.hypot(x2 - x1, y2 - y1)
                if edge_len >= helper_edge_length_mm:
                    mx = (x1 + x2) * 0.5
                    my = (y1 + y2) * 0.5
                    node_set.add((round(mx, 3), round(my, 3)))

        return [np.array((x, y), dtype=float) for x, y in node_set]

    @staticmethod
    def _point_is_free(point_xy: np.ndarray, obstacles) -> bool:
        pt = Point(float(point_xy[0]), float(point_xy[1]))
        return not any(poly.contains(pt) for poly in obstacles)

    @staticmethod
    def _segment_is_free(a_xy: np.ndarray, b_xy: np.ndarray, obstacles) -> bool:
        line = LineString([(float(a_xy[0]), float(a_xy[1])), (float(b_xy[0]), float(b_xy[1]))])
        for poly in obstacles:
            if line.crosses(poly) or line.within(poly) or poly.contains(line):
                return False
        return True

    @classmethod
    def _nearest_free_point(cls, point_xy: np.ndarray, obstacles) -> np.ndarray:
        if cls._point_is_free(point_xy, obstacles):
            return point_xy

        px, py = float(point_xy[0]), float(point_xy[1])
        angles = np.linspace(0.0, 2.0 * math.pi, 32, endpoint=False)
        for r in np.arange(2.0, 40.0, 2.0):
            for theta in angles:
                q = np.array([px + r * math.cos(theta), py + r * math.sin(theta)], dtype=float)
                if cls._point_is_free(q, obstacles):
                    return q
        return point_xy

    @classmethod
    def _build_visibility_graph(cls, nodes: Sequence[np.ndarray], obstacles):
        graph = nx.Graph()
        for i, node in enumerate(nodes):
            graph.add_node(i, pos=node)

        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                a = nodes[i]
                b = nodes[j]
                if cls._segment_is_free(a, b, obstacles):
                    graph.add_edge(i, j, weight=float(np.linalg.norm(a - b)))
        return graph

    def plan(self, start_xy: Iterable[float], goal_xy: Iterable[float]) -> List[Point2]:
        start = self._nearest_free_point(np.array(tuple(start_xy), dtype=float), self.obstacles)
        goal = self._nearest_free_point(np.array(tuple(goal_xy), dtype=float), self.obstacles)

        if self._segment_is_free(start, goal, self.obstacles):
            return [(float(start[0]), float(start[1])), (float(goal[0]), float(goal[1]))]

        nodes = list(self.base_nodes) + [start, goal]
        graph = self.base_graph.copy()
        sid = len(nodes) - 2
        gid = len(nodes) - 1
        graph.add_node(sid, pos=start)
        graph.add_node(gid, pos=goal)

        for i, node in enumerate(nodes[:-2]):
            if self._segment_is_free(start, node, self.obstacles):
                graph.add_edge(sid, i, weight=float(np.linalg.norm(start - node)))
            if self._segment_is_free(goal, node, self.obstacles):
                graph.add_edge(gid, i, weight=float(np.linalg.norm(goal - node)))

        if self._segment_is_free(start, goal, self.obstacles):
            graph.add_edge(sid, gid, weight=float(np.linalg.norm(start - goal)))

        try:
            index_path = nx.astar_path(
                graph,
                sid,
                gid,
                heuristic=lambda a, b: float(np.linalg.norm(graph.nodes[a]["pos"] - graph.nodes[b]["pos"])),
                weight="weight",
            )
        except nx.NetworkXNoPath:
            return [(float(start[0]), float(start[1]))]

        return [(float(graph.nodes[i]["pos"][0]), float(graph.nodes[i]["pos"][1])) for i in index_path]
