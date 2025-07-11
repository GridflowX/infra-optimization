import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import json
import random
from scipy.spatial import KDTree
from queue import PriorityQueue

# Configuration
WAREHOUSE_WIDTH = 1000
WAREHOUSE_HEIGHT = 10000
MIN_DIMENSION = 200
CLEARANCE = 20  # Minimum clearance between objects
ROADMAP_SIZE = 500  # Number of roadmap nodes

class Rectangle:
    def __init__(self, width, height, rid=None):
        self.width = width
        self.height = height
        self.rid = rid
        self.x = None
        self.y = None
        self.retrieval_path = None
    
    def get_bbox(self):
        return [self.x, self.y, self.x + self.width, self.y + self.height]
    
    def collides_with(self, other):
        if not other.x: return False
        return not (self.x + self.width + CLEARANCE <= other.x or
                    other.x + other.width + CLEARANCE <= self.x or
                    self.y + self.height + CLEARANCE <= other.y or
                    other.y + other.height + CLEARANCE <= self.y)
    
    def contains_point(self, px, py):
        return self.x <= px <= self.x + self.width and self.y <= py <= self.y + self.height

def generate_rectangles(num_rects):
    return [Rectangle(
        random.randint(MIN_DIMENSION, WAREHOUSE_WIDTH // 2),
        random.randint(MIN_DIMENSION, WAREHOUSE_HEIGHT // 10),
        i
    ) for i in range(num_rects)]

def pack_rectangles(rects):
    # Sort by area descending
    rects.sort(key=lambda r: r.width * r.height, reverse=True)
    
    # Initialize skyline and placements
    skyline = [(0, 0, WAREHOUSE_WIDTH)]
    placements = []
    max_height = 0
    
    for rect in rects:
        best_x, best_y = None, float('inf')
        
        # Find best placement position
        for i, (x, y, width) in enumerate(skyline):
            # Try placement without rotation
            if width >= rect.width:
                placement_y = y
                # Check collision with existing placements
                rect.x, rect.y = x, placement_y
                if not any(rect.collides_with(p) for p in placements):
                    if placement_y < best_y:
                        best_x, best_y = x, placement_y
            
            # Try placement with rotation
            if width >= rect.height:
                placement_y = y
                rect.width, rect.height = rect.height, rect.width
                rect.x, rect.y = x, placement_y
                if not any(rect.collides_with(p) for p in placements):
                    if placement_y < best_y:
                        best_x, best_y = x, placement_y
                rect.width, rect.height = rect.height, rect.width
        
        # Place rectangle if position found
        if best_x is not None:
            rect.x, rect.y = best_x, best_y
            placements.append(rect)
            max_height = max(max_height, best_y + rect.height)
            
            # Update skyline
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
            
            # Add new segment above placed rectangle
            new_skyline.append((best_x, best_y + rect.height, rect.width))
            skyline = new_skyline
    
    return placements, max_height

def build_roadmap(rects):
    nodes = []
    kd_tree = None
    
    # Generate collision-free nodes
    while len(nodes) < ROADMAP_SIZE:
        x = random.uniform(0, WAREHOUSE_WIDTH)
        y = random.uniform(0, WAREHOUSE_HEIGHT)
        
        # Reject if inside any rectangle
        if not any(rect.contains_point(x, y) for rect in rects):
            nodes.append((x, y))
            kd_tree = KDTree(nodes)  # Update KDTree
    
    # Create edges between nearby nodes
    edges = {}
    for i, (x, y) in enumerate(nodes):
        neighbors = kd_tree.query_ball_point([x, y], WAREHOUSE_WIDTH/10)
        edges[i] = [n for n in neighbors if n != i]
    
    return nodes, edges

def find_retrieval_path(rect, rects, roadmap):
    nodes, edges = roadmap
    start = (rect.x + rect.width/2, rect.y + rect.height/2)
    
    # Find closest roadmap node to rectangle center
    kd_tree = KDTree(nodes)
    start_idx = kd_tree.query([start])[1][0]
    
    # Find exit points on warehouse boundary
    exits = [
        (0, rect.y + rect.height/2),                  # Left
        (WAREHOUSE_WIDTH, rect.y + rect.height/2),     # Right
        (rect.x + rect.width/2, 0),                    # Bottom
        (rect.x + rect.width/2, WAREHOUSE_HEIGHT)      # Top
    ]
    
    # Try each exit point
    for exit_point in exits:
        path = a_star(nodes, edges, start_idx, exit_point, rects)
        if path:
            return [start] + path + [exit_point]
    return None

def a_star(nodes, edges, start_idx, goal, obstacles):
    open_set = PriorityQueue()
    open_set.put((0, start_idx))
    came_from = {}
    g_score = {i: float('inf') for i in range(len(nodes))}
    g_score[start_idx] = 0
    
    while not open_set.empty():
        _, current = open_set.get()
        current_pos = nodes[current]
        
        # Check if we can reach goal directly
        if can_move_directly(current_pos, goal, obstacles):
            return [nodes[i] for i in reconstruct_path(came_from, current)] + [goal]
        
        # Process neighbors
        for neighbor in edges[current]:
            neighbor_pos = nodes[neighbor]
            tentative_g = g_score[current] + np.linalg.norm(
                np.array(current_pos) - np.array(neighbor_pos))
            
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + np.linalg.norm(
                    np.array(neighbor_pos) - np.array(goal))
                open_set.put((f_score, neighbor))
    return None

def can_move_directly(start, end, obstacles):
    # Check straight-line path for collisions
    for rect in obstacles:
        if rect.x is None: continue
        if line_intersects_rect(start, end, rect):
            return False
    return True

def line_intersects_rect(p1, p2, rect):
    # Check if line segment intersects rectangle
    rect_bbox = [rect.x - CLEARANCE, rect.y - CLEARANCE,
                 rect.x + rect.width + CLEARANCE,
                 rect.y + rect.height + CLEARANCE]
    return line_intersects_bbox(p1, p2, rect_bbox)

def line_intersects_bbox(p1, p2, bbox):
    # Cohen-Sutherland line clipping algorithm
    x1, y1 = p1
    x2, y2 = p2
    xmin, ymin, xmax, ymax = bbox
    
    def compute_code(x, y):
        code = 0
        if x < xmin: code |= 1
        elif x > xmax: code |= 2
        if y < ymin: code |= 4
        elif y > ymax: code |= 8
        return code
    
    code1 = compute_code(x1, y1)
    code2 = compute_code(x2, y2)
    
    while True:
        if not (code1 | code2): return True
        if code1 & code2: return False
        
        code = code1 if code1 else code2
        if code & 1:  # Left
            x = xmin
            y = y1 + (y2 - y1) * (xmin - x1) / (x2 - x1)
        elif code & 2:  # Right
            x = xmax
            y = y1 + (y2 - y1) * (xmax - x1) / (x2 - x1)
        elif code & 4:  # Bottom
            y = ymin
            x = x1 + (x2 - x1) * (ymin - y1) / (y2 - y1)
        elif code & 8:  # Top
            y = ymax
            x = x1 + (x2 - x1) * (ymax - y1) / (y2 - y1)
        
        if code == code1:
            x1, y1 = x, y
            code1 = compute_code(x1, y1)
        else:
            x2, y2 = x, y
            code2 = compute_code(x2, y2)

def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.append(current)
    return path[::-1]

def visualize(rects, roadmap_nodes=None):
    fig, ax = plt.subplots(figsize=(12, 15))
    ax.set_xlim(0, WAREHOUSE_WIDTH)
    ax.set_ylim(0, WAREHOUSE_HEIGHT)
    
    # Draw warehouse
    ax.add_patch(patches.Rectangle(
        (0, 0), WAREHOUSE_WIDTH, WAREHOUSE_HEIGHT, 
        fill=False, edgecolor='black', linewidth=2))
    
    # Draw rectangles
    for rect in rects:
        if rect.x is not None:
            ax.add_patch(patches.Rectangle(
                (rect.x, rect.y), rect.width, rect.height,
                fill=True, alpha=0.7, edgecolor='blue'))
            ax.text(rect.x + rect.width/2, rect.y + rect.height/2, 
                   str(rect.rid), ha='center', va='center')
            
            # Draw retrieval path
            if rect.retrieval_path:
                path = np.array(rect.retrieval_path)
                ax.plot(path[:, 0], path[:, 1], 'r-', linewidth=1.5)
                ax.plot(path[0][0], path[0][1], 'go')  # Start
                ax.plot(path[-1][0], path[-1][1], 'ro')  # End
    
    # Draw roadmap (optional)
    if roadmap_nodes:
        nodes = np.array(roadmap_nodes)
        ax.plot(nodes[:, 0], nodes[:, 1], 'k.', markersize=2, alpha=0.5)
    
    plt.title('Warehouse Packing with Retrieval Paths')
    plt.savefig('warehouse_packing.png', dpi=150, bbox_inches='tight')
    plt.close()

def save_to_json(rects, filename):
    data = {
        "warehouse": {
            "width": WAREHOUSE_WIDTH,
            "height": WAREHOUSE_HEIGHT,
            "clearance": CLEARANCE
        },
        "placements": [
            {
                "id": rect.rid,
                "x": rect.x,
                "y": rect.y,
                "width": rect.width,
                "height": rect.height,
                "retrieval_path": rect.retrieval_path
            } for rect in rects if rect.x is not None
        ]
    }
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)

# Main execution
if __name__ == "__main__":
    # Generate random rectangles
    rectangles = generate_rectangles(15)
    
    # Pack rectangles
    placed_rects, used_height = pack_rectangles(rectangles)
    print(f"Packed {len(placed_rects)} rectangles, height used: {used_height:.1f}mm")
    
    # Build motion planning roadmap
    roadmap_nodes, roadmap_edges = build_roadmap(placed_rects)
    
    # Calculate retrieval paths
    for rect in placed_rects:
        rect.retrieval_path = find_retrieval_path(rect, placed_rects, 
                                                (roadmap_nodes, roadmap_edges))
    
    # Visualize and save results
    visualize(placed_rects, roadmap_nodes)
    save_to_json(placed_rects, 'packing_result.json')
    print("Results saved to warehouse_packing.png and packing_result.json")