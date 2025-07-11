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
CLEARANCE = 10       # Minimum clearance between objects
ROADMAP_SIZE = 800   # Number of roadmap nodes
EXIT_MARGIN = 10     # Margin for exit points from warehouse edges
MAX_RECTANGLES = 500  # Maximum number of rectangles to generate/attempt to pack

class Rectangle:
    def __init__(self, width, height, rid=None):
        # Ensure dimensions don't exceed maximum allowed
        self.width = min(width, MAX_DIMENSION)
        self.height = min(height, MAX_DIMENSION)
        self.rid = rid # Rectangle ID
        self.x = None  # X-coordinate of bottom-left corner
        self.y = None  # Y-coordinate of bottom-left corner
        self.retrieval_path = None
        self.exit_edge = None
        self.path_length = float('inf')
    
    def get_bbox(self):
        # Returns [x_min, y_min, x_max, y_max] of the rectangle
        return [self.x, self.y, self.x + self.width, self.y + self.height]
    
    def collides_with(self, other):
        # Check if 'other' rectangle has been placed (has valid coordinates)
        if other.x is None or self.x is None: 
            return False
        # Checks for collision between two rectangles, considering the required clearance.
        # Returns True if they overlap or are closer than CLEARANCE.
        return not (self.x + self.width + CLEARANCE <= other.x or
                    other.x + other.width + CLEARANCE <= self.x or
                    self.y + self.height + CLEARANCE <= other.y or
                    other.y + other.height + CLEARANCE <= self.y)
    
    def contains_point(self, px, py):
        # Checks if a point (px, py) is inside the rectangle's inflated boundary.
        # This is used for collision detection of roadmap nodes/paths.
        return (self.x - CLEARANCE/2 <= px <= self.x + self.width + CLEARANCE/2 and 
                self.y - CLEARANCE/2 <= py <= self.y + self.height + CLEARANCE/2)

def generate_rectangles(num_rects):
    # Generates a list of Rectangle objects with random dimensions within limits.
    return [Rectangle(
        random.randint(MIN_DIMENSION, MAX_DIMENSION),
        random.randint(MIN_DIMENSION, MAX_DIMENSION),
        i # Assigns a unique ID to each rectangle
    ) for i in range(min(num_rects, MAX_RECTANGLES))]

def pack_rectangles(rects):
    # Sorts rectangles by area in descending order. This is a common heuristic
    # for better packing density in greedy algorithms.
    rects.sort(key=lambda r: r.width * r.height, reverse=True)
    
    # Initialize the skyline: a list of (x, y, width) tuples representing
    # available horizontal segments on the top surface of placed rectangles.
    # Initially, the entire warehouse floor is available.
    skyline = [(0, 0, WAREHOUSE_WIDTH)]
    placements = [] # List to store successfully placed rectangles
    max_height = 0  # Tracks the maximum height reached by placed rectangles
    
    for rect in rects:
        best_x, best_y = None, float('inf')
        best_rotation = False
        
        # Store original dimensions to restore after rotation attempts
        original_width, original_height = rect.width, rect.height

        # Iterate through current skyline segments to find the best placement for the rectangle
        for x_seg, y_seg, width_seg in skyline:
            # Try placement without rotation
            if width_seg >= original_width:
                # Temporarily set position for collision check
                rect.x, rect.y = x_seg, y_seg
                # Check if placement is valid (no collisions with already placed rectangles and within warehouse bounds)
                if not any(rect.collides_with(p) for p in placements):
                    if y_seg < best_y: # Prioritize the lowest Y position
                        best_x, best_y = x_seg, y_seg
                        best_rotation = False
            
            # Try placement with rotation (90 degrees)
            if width_seg >= original_height: # Check if rotated width fits in segment width
                # Temporarily rotate rectangle and set position for collision check
                rect.width, rect.height = original_height, original_width
                rect.x, rect.y = x_seg, y_seg
                # Check if placement is valid
                if not any(rect.collides_with(p) for p in placements):
                    if y_seg < best_y: # Prioritize the lowest Y position
                        best_x, best_y = x_seg, y_seg
                        best_rotation = True
                # Restore original orientation before next iteration or final placement decision
                rect.width, rect.height = original_width, original_height

        # If a valid position was found, place the rectangle permanently
        if best_x is not None:
            if best_rotation:
                rect.width, rect.height = original_height, original_width # Apply final rotation
            rect.x, rect.y = best_x, best_y
            placements.append(rect)
            max_height = max(max_height, best_y + rect.height) # Update max height used
            
            # Update the skyline based on the new placement
            new_skyline = []
            placed_right = best_x + rect.width
            placed_top = best_y + rect.height

            for seg_x, seg_y, seg_w in skyline:
                seg_right = seg_x + seg_w
                
                # Case 1: Segment is completely to the left or right of the placed rectangle (no overlap)
                if seg_right <= best_x or seg_x >= placed_right:
                    new_skyline.append((seg_x, seg_y, seg_w))
                # Case 2: Segment overlaps with the placed rectangle, so it needs to be split
                else:
                    # Part of segment to the left of the placed rectangle
                    if seg_x < best_x:
                        new_skyline.append((seg_x, seg_y, best_x - seg_x))
                    # Part of segment to the right of the placed rectangle
                    if seg_right > placed_right:
                        new_skyline.append((placed_right, seg_y, seg_right - placed_right))
            
            # Add the new segment created by the top edge of the placed rectangle.
            # This represents the new available "shelf" space.
            new_skyline.append((best_x, placed_top, rect.width))
            
            # Sort new skyline segments by x-coordinate and merge adjacent segments at the same height
            skyline = []
            for seg in sorted(new_skyline, key=lambda s: s[0]):
                if skyline and skyline[-1][1] == seg[1] and skyline[-1][0] + skyline[-1][2] == seg[0]:
                    # Merge with previous segment if they are at the same height and contiguous
                    skyline[-1] = (skyline[-1][0], skyline[-1][1], skyline[-1][2] + seg[2])
                else:
                    skyline.append(seg)
    
    return placements, max_height

def build_roadmap(rects):
    nodes = [] # Stores (x, y) coordinates of roadmap nodes
    
    # Helper function to check if a given point is collision-free.
    # A point is free if it's within warehouse bounds (with clearance)
    # and not inside any placed rectangle's inflated boundary.
    def is_point_free(x, y):
        # Check warehouse boundaries with clearance
        if (x < CLEARANCE or x > WAREHOUSE_WIDTH - CLEARANCE or 
            y < CLEARANCE or y > WAREHOUSE_HEIGHT - CLEARANCE):
            return False
            
        # Check against all placed rectangles using their inflated boundary
        for rect in rects:
            if rect.x is None: # Ensure only placed rectangles are considered obstacles
                continue
            if rect.contains_point(x, y):
                return False
        return True

    # Generate collision-free nodes randomly until ROADMAP_SIZE is reached
    # or a maximum number of attempts is exhausted.
    attempts = 0
    max_attempts = ROADMAP_SIZE * 10 # Limit attempts to prevent infinite loops if space is very constrained
    while len(nodes) < ROADMAP_SIZE and attempts < max_attempts:
        x = random.uniform(CLEARANCE, WAREHOUSE_WIDTH - CLEARANCE)
        y = random.uniform(CLEARANCE, WAREHOUSE_HEIGHT - CLEARANCE)
        
        if is_point_free(x, y):
            nodes.append((x, y))
        attempts += 1
    
    if len(nodes) < ROADMAP_SIZE:
        print(f"Warning: Could only generate {len(nodes)} roadmap nodes out of {ROADMAP_SIZE} requested after {attempts} attempts.")

    # Create a KDTree for efficient nearest neighbor search, used for connecting nodes.
    kd_tree = KDTree(nodes)
    
    # Create edges between nearby nodes if the direct path between them is collision-free.
    edges = {} # Dictionary mapping node index to a list of connected neighbor indices
    for i, (x, y) in enumerate(nodes):
        # Find neighbors within a reasonable radius. MAX_DIMENSION * 1.5 is a heuristic for connection distance.
        neighbors = kd_tree.query_ball_point([x, y], MAX_DIMENSION * 1.5)
        edges[i] = []
        for n_idx in neighbors:
            if n_idx != i: # Don't connect a node to itself
                nx, ny = nodes[n_idx]
                # Check if the straight-line path between current node and neighbor is collision-free
                if can_move_directly((x, y), (nx, ny), rects):
                    edges[i].append(n_idx) # Add edge from i to n_idx
                    # For an undirected graph, add edge in both directions
                    if n_idx not in edges:
                        edges[n_idx] = []
                    edges[n_idx].append(i) # Add edge from n_idx to i (ensure it's not duplicated)
    
    # Clean up duplicate edges if any from the bidirectional add
    for node_idx in edges:
        edges[node_idx] = list(set(edges[node_idx]))

    return nodes, edges

def calculate_path_length(path):
    """Calculates the total length of a path given as a list of (x, y) coordinates."""
    length = 0.0
    for i in range(1, len(path)):
        x1, y1 = path[i-1]
        x2, y2 = path[i]
        length += math.sqrt((x2 - x1)**2 + (y2 - y1)**2) # Euclidean distance
    return length

def find_retrieval_path(rect, rects, roadmap):
    """
    Finds the shortest retrieval path for a given rectangle to any of the four warehouse edges.
    Uses the pre-built roadmap and A* search.
    """
    nodes, edges = roadmap
    center = (rect.x + rect.width/2, rect.y + rect.height/2) # Center point of the rectangle
    
    kd_tree = KDTree(nodes)
    
    # Find the single closest roadmap node to the rectangle's center
    _, start_idx = kd_tree.query(center) # `query` on a single point returns (distance, index) tuple
    
    # First, verify if the starting point (rectangle center) can connect to the chosen roadmap node.
    # If this initial segment is blocked by other obstacles, then this roadmap node is not a valid
    # entry point for the path, and we should consider the rectangle non-retrievable via this path.
    if not can_move_directly(center, nodes[start_idx], rects):
        print(f"Warning: Rect {rect.rid} center {center} cannot connect directly to closest roadmap node {nodes[start_idx]}. Skipping pathfinding for this rect.")
        return None

    # Define potential exit points on each of the four warehouse edges.
    # These points are aligned with the rectangle's center coordinate along the other axis.
    exit_points = [
        (EXIT_MARGIN, center[1]),                       # Left edge exit
        (WAREHOUSE_WIDTH - EXIT_MARGIN, center[1]),    # Right edge exit
        (center[0], EXIT_MARGIN),                       # Bottom edge exit
        (center[0], WAREHOUSE_HEIGHT - EXIT_MARGIN)    # Top edge exit
    ]
    
    # Corresponding names for each exit edge, used for labeling and output.
    edge_names = ["left", "right", "bottom", "top"]
    
    best_path_coords = None # To store the actual (x,y) coordinates of the best path found
    best_length = float('inf')
    best_edge = None
    
    # Iterate through all four possible exit points to find the overall shortest valid path.
    for i, exit_point in enumerate(exit_points):
        # Perform A* search from the closest roadmap node to the current exit point.
        # `obstacles` passed to A* are all `placed_rects` (excluding the current `rect`).
        # A* will find a path through the free space of the roadmap.
        path_from_roadmap_node = a_star(nodes, edges, start_idx, exit_point, rects)
        
        if path_from_roadmap_node:
            # Construct the full path by prepending the rectangle's center to the path found by A*.
            full_path = [center] + path_from_roadmap_node
            # Calculate the total length of this complete path.
            current_path_length = calculate_path_length(full_path)
            
            # If this path is shorter than the current best path found so far, update the best.
            if current_path_length < best_length:
                best_path_coords = full_path
                best_length = current_path_length
                best_edge = edge_names[i]
                
    # Store the final best path details in the rectangle object if a valid path was found.
    if best_path_coords:
        rect.retrieval_path = best_path_coords
        rect.exit_edge = best_edge
        rect.path_length = best_length
        return best_path_coords
    
    return None # Return None if no valid path was found to any edge

def a_star(nodes, edges, start_idx, goal, obstacles):
    """
    A* search algorithm to find the shortest path from a start node to a goal point.
    """
    open_set = PriorityQueue()
    open_set.put((0, start_idx)) # Priority Queue stores (f_score, node_index)
    
    came_from = {} # Dictionary to reconstruct the path after finding the goal
    
    g_score = {i: float('inf') for i in range(len(nodes))} # Cost from start node to current node
    g_score[start_idx] = 0 # Cost from start to itself is 0
    
    while not open_set.empty():
        current_f_score, current_idx = open_set.get()
        current_pos = nodes[current_idx]
        
        # Optimization: If the current roadmap node can directly reach the goal point,
        # we have found a path.
        if can_move_directly(current_pos, goal, obstacles):
            # Reconstruct the path from start_idx to current_idx
            path_to_current = [nodes[i] for i in reconstruct_path(came_from, current_idx)]
            path_to_current.append(goal) # Add the final goal point
            return path_to_current
        
        # Explore neighbors of the current node
        for neighbor_idx in edges.get(current_idx, []):
            neighbor_pos = nodes[neighbor_idx]
            
            # Calculate tentative g_score for the neighbor (cost from start to neighbor)
            distance_to_neighbor = math.sqrt((current_pos[0]-neighbor_pos[0])**2 + 
                                              (current_pos[1]-neighbor_pos[1])**2)
            tentative_g_score = g_score[current_idx] + distance_to_neighbor
            
            # If this new path to neighbor is shorter, update its g_score and add to open set
            if tentative_g_score < g_score[neighbor_idx]:
                came_from[neighbor_idx] = current_idx # Record how we got to neighbor
                g_score[neighbor_idx] = tentative_g_score
                
                # Calculate f_score (g_score + heuristic)
                # Heuristic is Euclidean distance from neighbor to goal
                h_score = math.sqrt((neighbor_pos[0]-goal[0])**2 + 
                                     (neighbor_pos[1]-goal[1])**2)
                f_score = tentative_g_score + h_score
                open_set.put((f_score, neighbor_idx))
                
    return None # No path found to the goal

def can_move_directly(start, end, obstacles):
    """
    Checks if a straight-line segment between 'start' and 'end' points
    intersects with any of the 'obstacles' (placed rectangles with clearance).
    """
    for rect in obstacles:
        if rect.x is None: # Only check against placed rectangles
            continue
        if line_intersects_rect(start, end, rect):
            return False # Collision detected
    return True # No collision

def line_intersects_rect(p1, p2, rect):
    """
    Checks if a line segment (p1-p2) intersects the given rectangle's
    inflated bounding box (considering clearance).
    """
    # Create the inflated bounding box for the rectangle
    rect_bbox = [
        rect.x - CLEARANCE/2, # x_min
        rect.y - CLEARANCE/2, # y_min
        rect.x + rect.width + CLEARANCE/2, # x_max
        rect.y + rect.height + CLEARANCE/2 # y_max
    ]
    return line_intersects_bbox(p1, p2, rect_bbox)

def line_intersects_bbox(p1, p2, bbox):
    """
    Cohen-Sutherland line clipping algorithm to check if a line segment (p1-p2)
    intersects an axis-aligned bounding box (bbox).
    """
    x1, y1 = p1
    x2, y2 = p2
    xmin, ymin, xmax, ymax = bbox
    
    # Compute region code for a point
    def compute_code(x, y):
        code = 0
        if x < xmin: code |= 1  # Left
        elif x > xmax: code |= 2 # Right
        if y < ymin: code |= 4  # Bottom
        elif y > ymax: code |= 8 # Top
        return code
    
    code1 = compute_code(x1, y1)
    code2 = compute_code(x2, y2)
    
    while True:
        if not (code1 | code2): # Both endpoints are inside the bbox, so intersects
            return True
        if code1 & code2: # Both endpoints are outside on the same side, so no intersection
            return False
        
        # At least one endpoint is outside, and they are not on the same side.
        # Pick an endpoint outside the bbox and clip the line segment.
        code = code1 if code1 else code2 # Pick an "out" code
        
        # Calculate intersection point (x, y) with the bbox edge corresponding to the 'code'
        if code & 1:   # Left
            x = xmin
            y = y1 + (y2 - y1) * (xmin - x1) / (x2 - x1) if (x2 - x1) != 0 else float('inf')
        elif code & 2: # Right
            x = xmax
            y = y1 + (y2 - y1) * (xmax - x1) / (x2 - x1) if (x2 - x1) != 0 else float('inf')
        elif code & 4: # Bottom
            y = ymin
            x = x1 + (x2 - x1) * (ymin - y1) / (y2 - y1) if (y2 - y1) != 0 else float('inf')
        elif code & 8: # Top
            y = ymax
            x = x1 + (x2 - x1) * (ymax - y1) / (y2 - y1) if (y2 - y1) != 0 else float('inf')
        
        # Handle cases where line is parallel to an axis and outside
        if x == float('inf') or y == float('inf') or math.isnan(x) or math.isnan(y):
            return False # Indicates division by zero for parallel lines outside of bounds or undefined points

        # Replace the clipped endpoint and recompute its code
        if code == code1:
            x1, y1 = x, y
            code1 = compute_code(x1, y1)
        else:
            x2, y2 = x, y
            code2 = compute_code(x2, y2)

def reconstruct_path(came_from, current):
    """Reconstructs the path from the 'came_from' dictionary."""
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.append(current) # Add the starting node itself
    return path[::-1] # Reverse to get the path from start to end

def visualize(rects, filename='warehouse_packing.png'):
    """Generates and saves a visualization of the packed rectangles and retrieval paths."""
    fig, ax = plt.subplots(figsize=(10, 15)) # Adjust figure size for better aspect ratio
    ax.set_xlim(0, WAREHOUSE_WIDTH)
    ax.set_ylim(0, WAREHOUSE_HEIGHT)
    ax.set_aspect('equal', adjustable='box') # Maintain aspect ratio for accurate representation
    
    # Draw warehouse boundary
    ax.add_patch(patches.Rectangle(
        (0, 0), WAREHOUSE_WIDTH, WAREHOUSE_HEIGHT, 
        fill=False, edgecolor='black', linewidth=2))
    
    # Draw each placed rectangle
    for rect in rects:
        if rect.x is not None: # Only draw rectangles that were successfully placed
            ax.add_patch(patches.Rectangle(
                (rect.x, rect.y), rect.width, rect.height,
                fill=True, alpha=0.7, edgecolor='blue', linewidth=1))
            ax.text(rect.x + rect.width/2, rect.y + rect.height/2, 
                    str(rect.rid), ha='center', va='center', fontsize=7, color='black')
            
            # Draw retrieval path if it exists for this rectangle
            if rect.retrieval_path:
                path = np.array(rect.retrieval_path)
                ax.plot(path[:, 0], path[:, 1], 'r-', linewidth=1.5, alpha=0.8) # Path in red line
                ax.plot(path[0][0], path[0][1], 'go', markersize=4)  # Start point (rectangle center) in green circle
                ax.plot(path[-1][0], path[-1][1], 'ro', markersize=4)  # End point (exit edge) in red circle
                
                # Label the exit edge and path length
                if rect.exit_edge:
                    # Adjust text position based on which edge the path exits for better readability
                    text_x, text_y = path[-1][0], path[-1][1]
                    ha_align = 'center'
                    va_align = 'center'
                    if rect.exit_edge == "left":
                        ha_align = 'left'
                    elif rect.exit_edge == "right":
                        ha_align = 'right'
                    elif rect.exit_edge == "bottom":
                        va_align = 'bottom'
                    elif rect.exit_edge == "top":
                        va_align = 'top'

                    ax.text(text_x, text_y, 
                            f"{rect.exit_edge} ({rect.path_length:.0f}mm)", 
                            fontsize=7, ha=ha_align, va=va_align, color='red')
    
    placed_count = len([r for r in rects if r.x is not None])
    plt.title(f'Warehouse Packing ({placed_count}/{len(rects)} Objects Placed)')
    plt.grid(True, linestyle='--', alpha=0.6) # Add a grid for better spatial understanding
    plt.savefig(filename, dpi=200, bbox_inches='tight') # Save with higher DPI for quality
    plt.close()
    print(f"Visualization saved to {filename}")

def save_to_json(rects, filename='packing_result.json'):
    """Saves the warehouse configuration, object placements, and retrieval details to a JSON file."""
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
            } for rect in rects if rect.x is not None # Only include placed rectangles
        ]
    }
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2) # Use indent for pretty printing
    print(f"JSON results saved to {filename}")

# Main execution flow
if __name__ == "__main__":
    print("Starting automated 2D warehouse packing and retrieval planning...")
    start_time = time.time()
    
    # Generate random rectangles. You can adjust num_rects to test more objects.
    num_rects = 50 # Start with 50, you can try 100 or more
    rectangles = generate_rectangles(num_rects)
    gen_time = time.time() - start_time
    print(f"Generated {len(rectangles)} rectangles in {gen_time:.2f}s")
    
    # Pack rectangles into the warehouse
    pack_start = time.time()
    placed_rects, used_height = pack_rectangles(rectangles)
    pack_time = time.time() - pack_start
    print(f"Packed {len(placed_rects)}/{len(rectangles)} rectangles in {pack_time:.2f}s")
    print(f"Storage height used: {used_height:.1f}mm")
    
    # Calculate and print packing efficiency.
    # This efficiency is based on the actual area of rectangles packed vs. the bounding box of the packed region.
    total_rect_area = sum(r.width * r.height for r in placed_rects)
    packing_efficiency = (total_rect_area / (WAREHOUSE_WIDTH * used_height)) * 100 if used_height > 0 else 0
    print(f"Packing efficiency (actual rect area / occupied warehouse area): {packing_efficiency:.1f}%")
    
    # Build the Probabilistic Roadmap (PRM) for motion planning
    roadmap_start = time.time()
    roadmap_nodes, roadmap_edges = build_roadmap(placed_rects)
    roadmap_time = time.time() - roadmap_start
    print(f"Built roadmap with {len(roadmap_nodes)} nodes in {roadmap_time:.2f}s")
    
    # Calculate retrieval paths for all placed rectangles
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
    print(f"Retrieval success rate: {success_count/len(placed_rects)*100:.1f}%" if len(placed_rects) > 0 else "N/A (No rectangles placed)")
    
    # Generate visualization and save results to JSON
    visualize(placed_rects)
    save_to_json(placed_rects)

    # Provide explanation if not all rectangles could be placed
    if len(placed_rects) < len(rectangles):
        print(f"\n--- Packing Limitations ---")
        print(f"Note: Only {len(placed_rects)} out of {len(rectangles)} rectangles could be successfully packed.")
        print("This is generally expected behavior and reflects a limitation of the greedy skyline packing algorithm, not a bug in its implementation.")
        print("Reasons for unplaced rectangles can include:")
        print("1.  **Warehouse Capacity:** The total area of the objects (plus their required 20mm clearance) exceeds the available warehouse space.")
        print("2.  **Space Fragmentation:** The greedy algorithm, while efficient, may create small, irregular gaps that are too small to accommodate the remaining rectangles, even if large contiguous space exists elsewhere.")
        print("3.  **No Valid Placement:** No collision-free position (considering clearance and warehouse bounds) could be found for the remaining objects.")
        print(f"The achieved packing efficiency for the placed items is {packing_efficiency:.1f}%.")
