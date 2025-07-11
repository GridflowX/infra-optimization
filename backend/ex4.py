import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import json
import random
from scipy.spatial import KDTree
from queue import PriorityQueue
import math

# Configuration
WAREHOUSE_WIDTH = 1000
WAREHOUSE_HEIGHT = 10000
MAX_DIMENSION = 200
MIN_DIMENSION = 50
CLEARANCE = 20
ROADMAP_SIZE = 1000

class Rectangle:
    def __init__(self, width, height, rid=None):
        self.width = min(width, MAX_DIMENSION)
        self.height = min(height, MAX_DIMENSION)
        self.rid = rid
        self.x = None
        self.y = None
        self.retrieval_path = None
        self.exit_edge = None
        self.path_length = float('inf')

    def get_bbox(self):
        return [self.x, self.y, self.x + self.width, self.y + self.height]

    def collides_with(self, other):
        if other.x is None: return False
        return not (self.x + self.width + CLEARANCE <= other.x or
                    other.x + other.width + CLEARANCE <= self.x or
                    self.y + self.height + CLEARANCE <= other.y or
                    other.y + other.height + CLEARANCE <= self.y)

    def contains_point(self, px, py):
        return (self.x - CLEARANCE/2 <= px <= self.x + self.width + CLEARANCE/2 and
                self.y - CLEARANCE/2 <= py <= self.y + self.height + CLEARANCE/2)

def generate_rectangles(num_rects):
    return [Rectangle(
        random.randint(MIN_DIMENSION, MAX_DIMENSION),
        random.randint(MIN_DIMENSION, MAX_DIMENSION),
        i) for i in range(num_rects)]

def pack_rectangles(rects):
    rects.sort(key=lambda r: r.width * r.height, reverse=True)
    skyline = [(0, 0, WAREHOUSE_WIDTH)]
    placements = []
    max_height = 0

    for rect in rects:
        best_x, best_y = None, float('inf')
        best_rotation = False

        for x, y, width in skyline:
            for rotation in [False, True]:
                w, h = (rect.height, rect.width) if rotation else (rect.width, rect.height)
                if width >= w and y < best_y:
                    rect.x, rect.y = x, y
                    rect.width, rect.height = w, h
                    if not any(rect.collides_with(p) for p in placements):
                        best_x, best_y = x, y
                        best_rotation = rotation

        if best_x is not None:
            if best_rotation:
                rect.width, rect.height = rect.height, rect.width
            rect.x, rect.y = best_x, best_y
            placements.append(rect)
            max_height = max(max_height, best_y + rect.height)

            new_skyline = []
            placed_right = best_x + rect.width
            for seg in skyline:
                seg_x, seg_y, seg_w = seg
                seg_right = seg_x + seg_w

                if seg_right <= best_x or seg_x >= placed_right:
                    new_skyline.append(seg)
                else:
                    if seg_x < best_x:
                        new_skyline.append((seg_x, seg_y, best_x - seg_x))
                    if seg_right > placed_right:
                        new_skyline.append((placed_right, seg_y, seg_right - placed_right))

            new_skyline.append((best_x, best_y + rect.height, rect.width))
            skyline = merge_skyline(new_skyline)
        else:
            print(f"Could not place rectangle {rect.rid} ({rect.width}×{rect.height})")

    return placements, max_height

def merge_skyline(skyline):
    if not skyline:
        return []
    merged = [skyline[0]]
    for seg in skyline[1:]:
        last = merged[-1]
        if last[1] == seg[1] and last[0] + last[2] == seg[0]:
            merged[-1] = (last[0], last[1], last[2] + seg[2])
        else:
            merged.append(seg)
    return merged

def can_move_directly(p1, p2, rects):
    steps = 10
    for i in range(1, steps):
        px = p1[0] + (p2[0] - p1[0]) * i / steps
        py = p1[1] + (p2[1] - p1[1]) * i / steps
        for rect in rects:
            if rect.contains_point(px, py):
                return False
    return True

def build_roadmap(rects):
    nodes = []
    def is_free(x, y):
        if (x < CLEARANCE or x > WAREHOUSE_WIDTH - CLEARANCE or
            y < CLEARANCE or y > WAREHOUSE_HEIGHT - CLEARANCE):
            return False
        for r in rects:
            if r.contains_point(x, y): return False
        return True

    while len(nodes) < ROADMAP_SIZE:
        x = random.uniform(CLEARANCE, WAREHOUSE_WIDTH - CLEARANCE)
        y = random.uniform(CLEARANCE, WAREHOUSE_HEIGHT - CLEARANCE)
        if is_free(x, y):
            nodes.append((x, y))

    kd = KDTree(nodes)
    edges = {}
    for i, (x, y) in enumerate(nodes):
        neighbors = kd.query_ball_point([x, y], MAX_DIMENSION * 3)
        edges[i] = [j for j in neighbors if j != i and can_move_directly((x, y), nodes[j], rects)]
    return nodes, edges

def a_star(nodes, edges, start_idx, goal_point, rects):
    goal_idx = len(nodes)
    nodes.append(goal_point)
    edges[goal_idx] = []
    for i, node in enumerate(nodes[:-1]):
        if can_move_directly(node, goal_point, rects):
            edges[i].append(goal_idx)
            edges[goal_idx].append(i)

    queue = PriorityQueue()
    queue.put((0, start_idx, []))
    visited = set()

    while not queue.empty():
        cost, current, path = queue.get()
        if current in visited:
            continue
        visited.add(current)
        path = path + [nodes[current]]
        if current == goal_idx:
            nodes.pop()
            edges.pop(goal_idx)
            return path

        for neighbor in edges[current]:
            if neighbor not in visited:
                dist = math.dist(nodes[current], nodes[neighbor])
                heuristic = math.dist(nodes[neighbor], goal_point)
                queue.put((cost + dist + heuristic, neighbor, path))

    nodes.pop()
    edges.pop(goal_idx)
    return None

def get_edge_name(point):
    if abs(point[0] - CLEARANCE) < 1e-2:
        return 'left'
    elif abs(point[0] - (WAREHOUSE_WIDTH - CLEARANCE)) < 1e-2:
        return 'right'
    elif abs(point[1] - CLEARANCE) < 1e-2:
        return 'bottom'
    elif abs(point[1] - (WAREHOUSE_HEIGHT - CLEARANCE)) < 1e-2:
        return 'top'
    return 'unknown'

def find_retrieval_path(rect, rects, roadmap):
    nodes, edges = roadmap
    center = (rect.x + rect.width / 2, rect.y + rect.height / 2)
    kd = KDTree(nodes)
    _, start_idx = kd.query([center])
    start_idx = start_idx[0]

    exit_points = [
        (CLEARANCE, center[1]),
        (WAREHOUSE_WIDTH - CLEARANCE, center[1]),
        (center[0], WAREHOUSE_HEIGHT - CLEARANCE),
        (center[0], CLEARANCE)
    ]

    best_path = None
    best_length = float('inf')
    best_edge = None

    for exit_point in exit_points:
        path = a_star(nodes[:], {k: v[:] for k, v in edges.items()}, start_idx, exit_point, rects)
        if path:
            length = sum(math.dist(path[i], path[i+1]) for i in range(len(path)-1))
            if length < best_length:
                best_path = [center] + path
                best_length = length
                best_edge = get_edge_name(exit_point)

    if best_path:
        rect.retrieval_path = best_path
        rect.exit_edge = best_edge
        rect.path_length = best_length

    return best_path

def visualize(rects, filename='warehouse_packing.png'):
    fig, ax = plt.subplots(figsize=(10, 15))
    ax.set_xlim(0, WAREHOUSE_WIDTH)
    ax.set_ylim(0, WAREHOUSE_HEIGHT)

    for rect in rects:
        if rect.x is not None:
            color = 'blue' if rect.retrieval_path else 'red'
            ax.add_patch(patches.Rectangle(
                (rect.x, rect.y), rect.width, rect.height,
                fill=True, alpha=0.7, edgecolor=color))
            ax.text(rect.x + rect.width/2, rect.y + rect.height/2, 
                    str(rect.rid), ha='center', va='center', fontsize=6)
            if rect.retrieval_path:
                path = np.array(rect.retrieval_path)
                ax.plot(path[:, 0], path[:, 1], 'r-', linewidth=1)
                ax.plot(path[0][0], path[0][1], 'go', markersize=4)
                ax.plot(path[-1][0], path[-1][1], 'ro', markersize=4)
                ax.text(path[-1][0], path[-1][1], 
                        f"{rect.exit_edge} ({rect.path_length:.0f}mm)", 
                        fontsize=6, ha='right', va='bottom')

    plt.title(f'Warehouse Packing ({len([r for r in rects if r.x is not None])} Placed)')
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Visualization saved to {filename}")

def save_to_json(rects, filename='packing_result.json'):
    data = []
    for r in rects:
        if r.x is not None:
            data.append({
                'id': r.rid,
                'x': r.x,
                'y': r.y,
                'width': r.width,
                'height': r.height,
                'exit_edge': r.exit_edge,
                'path_length': r.path_length,
                'retrieval_path': r.retrieval_path
            })
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"Saved packing result to {filename}")

if __name__ == '__main__':
    num_rects = 50
    rectangles = generate_rectangles(num_rects)
    placed_rects, used_height = pack_rectangles(rectangles)
    print(f"Placed {len(placed_rects)}/{num_rects} rectangles")
    print(f"Height used: {used_height}mm ({used_height / WAREHOUSE_HEIGHT * 100:.1f}%)")

    roadmap = build_roadmap(placed_rects)
    success = 0
    for r in placed_rects:
        if find_retrieval_path(r, placed_rects, roadmap):
            success += 1
    print(f"Found retrieval paths for {success}/{len(placed_rects)} rectangles")
    visualize(placed_rects)
    save_to_json(placed_rects)

