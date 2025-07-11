import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import json
import random
from scipy.spatial import KDTree
from queue import PriorityQueue
import math
import time

# Configuration
WAREHOUSE_WIDTH = 1000
WAREHOUSE_HEIGHT = 10000
MAX_DIMENSION = 200  # Maximum dimension for any object
MIN_DIMENSION = 50   # Minimum dimension for objects
CLEARANCE = 20       # Minimum clearance between objects
ROADMAP_SIZE = 800   # Number of roadmap nodes
EXIT_MARGIN = 50     # Margin for exit points
MAX_RECTANGLES = 100  # Maximum number of rectangles to pack

class Rectangle:
    def __init__(self, width, height, rid=None):
        # Ensure dimensions don't exceed maximum
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
        if not other.x: 
            return False
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
        i
    ) for i in range(min(num_rects, MAX_RECTANGLES))]

def pack_rectangles(rects):
    # Sort by area descending
    rects.sort(key=lambda r: r.width * r.height, reverse=True)
    
    # Initialize skyline and placements
    skyline = [(0, 0, WAREHOUSE_WIDTH)]
    placements = []
    max_height = 0
    
    for rect in rects:
        best_x, best_y = None, float('inf')
        best_rotation = False
        
        # Find best placement position
        for i, (x, y, width) in enumerate(skyline):
            # Try placement without rotation
            if width >= rect.width and y < best_y:
                # Check if placement would be valid
                rect.x, rect.y = x, y
                if not any(rect.collides_with(p) for p in placements):
                    best_x, best_y = x, y
                    best_rotation = False
            
            # Try placement with rotation
            if width >= rect.height and y < best_y:
                # Temporarily rotate
                rect.width, rect.height = rect.height, rect.width
                rect.x, rect.y = x, y
                if not any(rect.collides_with(p) for p in placements):
                    best_x, best_y = x, y
                    best_rotation = True
                # Restore original orientation
                rect.width, rect.height = rect.height, rect.width
        
        # Place rectangle if position found
        if best_x is not None:
            if best_rotation:
                rect.width, rect.height = rect.height, rect.width
            rect.x, rect.y = best_x, best_y
            placements.append(rect)
            max_height = max(max_height, best_y + rect.height)
            
            # Update skyline
            new_skyline = []
            placed_right = best_x + rect.width
            for seg in skyline:
                seg_x, seg_y, seg_w = seg
                seg_right = seg_x + seg_w
                
                # Check if segment overlaps with placed rectangle
                if seg_right <= best_x or seg_x >= placed_right:
                    new_skyline.append(seg)
                else:
                    # Split segment around placed rectangle
                    if seg_x < best_x:
                        new_skyline.append((seg_x, seg_y, best_x - seg_x))
                    if seg_right > placed_right:
                        new_skyline.append((placed_right, seg_y, seg_right - placed_right))
            
            # Add new segment above placed rectangle
            new_skyline.append((best_x, best_y + rect.height, rect.width))
            
            # Merge adjacent segments at same height
            skyline = []
            for seg in sorted(new_skyline, key=lambda s: s[0]):
                if skyline and skyline[-1][1] == seg[1] and skyline[-1][0] + skyline[-1][2] == seg[0]:
                    # Merge with previous segment
                    skyline[-1] = (skyline[-1][0], skyline[-1][1], skyline[-1][2] + seg[2])
                else:
                    skyline.append(seg)
    
    return placements, max_height

def build_roadmap(rects):
    nodes = []
    
    # Helper function to check if point is collision-free
    def is_point_free(x, y):
        # Check warehouse boundaries with clearance
        if (x < CLEARANCE or x > WAREHOUSE_WIDTH - CLEARANCE or 
            y < CLEARANCE or y > WAREHOUSE_HEIGHT - CLEARANCE):
            return False
            
        # Check against all rectangles with clearance
        for rect in rects:
            if rect.x is None: 
                continue
            if (x >= rect.x - CLEARANCE/2 and x <= rect.x + rect.width + CLEARANCE/2 and
                y >= rect.y - CLEARANCE/2 and y <= rect.y + rect.height + CLEARANCE/2):
                return False
        return True

    # Generate collision-free nodes
    while len(nodes) < ROADMAP_SIZE:
        x = random.uniform(CLEARANCE, WAREHOUSE_WIDTH - CLEARANCE)
        y = random.uniform(CLEARANCE, WAREHOUSE_HEIGHT - CLEARANCE)
        
        if is_point_free(x, y):
            nodes.append((x, y))
    
    # Create KDTree for efficient nearest neighbor search
    kd_tree = KDTree(nodes)
    
    # Create edges between nearby nodes
    edges = {}
    for i, (x, y) in enumerate(nodes):
        # Find neighbors within a reasonable radius
        neighbors = kd_tree.query_ball_point([x, y], MAX_DIMENSION * 2)
        edges[i] = []
        for n in neighbors:
            if n != i:
                # Check if direct path is collision-free
                nx, ny = nodes[n]
                if can_move_directly((x, y), (nx, ny), rects):
                    edges[i].append(n)
    
    return nodes, edges

def calculate_path_length(path):
    """Calculate total length of a path"""
    length = 0.0
    for i in range(1, len(path)):
        x1, y1 = path[i-1]
        x2, y2 = path[i]
        length += math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return length

def find_retrieval_path(rect, rects, roadmap):
    nodes, edges = roadmap
    center = (rect.x + rect.width/2, rect.y + rect.height/2)
    
    # Create KDTree for efficient searches
    kd_tree = KDTree(nodes)
    
    # Find closest roadmap node to rectangle center
    _, start_idx = kd_tree.query([center])
    start_idx = start_idx[0]
    
    # Define exit points on each edge with margin
    exit_points = [
        (EXIT_MARGIN, center[1]),                      # Left edge
        (WAREHOUSE_WIDTH - EXIT_MARGIN, center[1]),     # Right edge
        (center[0], EXIT_MARGIN),                      # Bottom edge
        (center[0], WAREHOUSE_HEIGHT - EXIT_MARGIN)     # Top edge
    ]
    
    # Edge names corresponding to exit points
    edge_names = ["left", "right", "bottom", "top"]
    
    best_path = None
    best_length = float('inf')
    best_edge = None
    
    # Try all exit points and select the shortest path
    for i, exit_point in enumerate(exit_points):
        # Skip edges that would require moving in the wrong direction
        if edge_names[i] == "left" and center[0] < WAREHOUSE_WIDTH/2:
            continue
        if edge_names[i] == "right" and center[0] > WAREHOUSE_WIDTH/2:
            continue
        if edge_names[i] == "bottom" and center[1] < WAREHOUSE_HEIGHT/2:
            continue
        if edge_names[i] == "top" and center[1] > WAREHOUSE_HEIGHT/2:
            continue
            
        path = a_star(nodes, edges, start_idx, exit_point, rects)
        if path:
            path_length = calculate_path_length(path)
            if path_length < best_length:
                best_path = [center] + path
                best_length = path_length
                best_edge = edge_names[i]
    
    # If no direct path found, try all edges without filtering
    if best_path is None:
        for i, exit_point in enumerate(exit_points):
            path = a_star(nodes, edges, start_idx, exit_point, rects)
            if path:
                path_length = calculate_path_length(path)
                if path_length < best_length:
                    best_path = [center] + path
                    best_length = path_length
                    best_edge = edge_names[i]
    
    if best_path:
        rect.retrieval_path = best_path
        rect.exit_edge = best_edge
        rect.path_length = best_length
        return best_path
    
    return None

def get_edge_name(point):
    x, y = point
    if x <= EXIT_MARGIN:
        return "left"
    elif x >= WAREHOUSE_WIDTH - EXIT_MARGIN:
        return "right"
    elif y <= EXIT_MARGIN:
        return "bottom"
    else:
        return "top"

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
            # Reconstruct path
            path = [nodes[i] for i in reconstruct_path(came_from, current)]
            path.append(goal)
            return path
        
        # Process neighbors
        for neighbor in edges.get(current, []):
            neighbor_pos = nodes[neighbor]
            # Calculate tentative g score
            distance = math.sqrt((current_pos[0]-neighbor_pos[0])**2 + 
                                (current_pos[1]-neighbor_pos[1])**2)
            tentative_g = g_score[current] + distance
            
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                # Calculate heuristic (Euclidean distance to goal)
                h_score = math.sqrt((neighbor_pos[0]-goal[0])**2 + 
                                   (neighbor_pos[1]-goal[1])**2)
                f_score = tentative_g + h_score
                open_set.put((f_score, neighbor))
    
    return None

def can_move_directly(start, end, obstacles):
    # Check straight-line path for collisions
    for rect in obstacles:
        if rect.x is None: 
            continue
        if line_intersects_rect(start, end, rect):
            return False
    return True

def line_intersects_rect(p1, p2, rect):
    # Check if line segment intersects rectangle with clearance
    rect_bbox = [
        rect.x - CLEARANCE/2, 
        rect.y - CLEARANCE/2,
        rect.x + rect.width + CLEARANCE/2,
        rect.y + rect.height + CLEARANCE/2
    ]
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
        if not (code1 | code2): 
            return True
        if code1 & code2: 
            return False
        
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

def visualize(rects, filename='warehouse_packing.png'):
    fig, ax = plt.subplots(figsize=(10, 15))
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
                   str(rect.rid), ha='center', va='center', fontsize=8)
            
            # Draw retrieval path if exists
            if rect.retrieval_path:
                path = np.array(rect.retrieval_path)
                ax.plot(path[:, 0], path[:, 1], 'r-', linewidth=1.5)
                ax.plot(path[0][0], path[0][1], 'go', markersize=6)  # Start
                ax.plot(path[-1][0], path[-1][1], 'ro', markersize=6)  # End
                
                # Label exit edge
                if rect.exit_edge:
                    ax.text(path[-1][0], path[-1][1], 
                           f"{rect.exit_edge} ({rect.path_length:.0f}mm)", 
                           fontsize=8, ha='right', va='bottom')
    
    plt.title(f'Warehouse Packing ({len(rects)} Objects)')
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Visualization saved to {filename}")

def save_to_json(rects, filename='packing_result.json'):
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
                "exit_edge": rect.exit_edge,
                "path_length": rect.path_length,
                "retrieval_path": rect.retrieval_path
            } for rect in rects if rect.x is not None
        ]
    }
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"JSON results saved to {filename}")

# Main execution
if __name__ == "__main__":
    print("Starting warehouse packing and retrieval planning...")
    start_time = time.time()
    
    # Generate random rectangles (max dimension 200mm)
    num_rects = 100
    rectangles = generate_rectangles(num_rects)
    gen_time = time.time() - start_time
    print(f"Generated {len(rectangles)} rectangles in {gen_time:.2f}s")
    
    # Pack rectangles
    pack_start = time.time()
    placed_rects, used_height = pack_rectangles(rectangles)
    pack_time = time.time() - pack_start
    print(f"Packed {len(placed_rects)}/{len(rectangles)} rectangles in {pack_time:.2f}s")
    print(f"Storage height used: {used_height:.1f}mm")
    print(f"Storage efficiency: {used_height/WAREHOUSE_HEIGHT*100:.1f}%")
    
    # Build motion planning roadmap
    roadmap_start = time.time()
    roadmap_nodes, roadmap_edges = build_roadmap(placed_rects)
    roadmap_time = time.time() - roadmap_start
    print(f"Built roadmap with {len(roadmap_nodes)} nodes in {roadmap_time:.2f}s")
    
    # Calculate retrieval paths
    path_start = time.time()
    success_count = 0
    for rect in placed_rects:
        path = find_retrieval_path(rect, placed_rects, (roadmap_nodes, roadmap_edges))
        if path:
            success_count += 1
    
    path_time = time.time() - path_start
    total_time = time.time() - start_time
    print(f"Found retrieval paths for {success_count}/{len(placed_rects)} rectangles in {path_time:.2f}s")
    print(f"Total execution time: {total_time:.2f}s")
    
    # Visualize and save results
    visualize(placed_rects)
    save_to_json(placed_rects)
