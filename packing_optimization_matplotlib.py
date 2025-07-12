import random
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation, FFMpegWriter
from collections import deque
import csv

# --- Parameters ---
STORAGE_WIDTH = 5000
STORAGE_LENGTH = 10000
NUM_RECTS = 3000
MIN_SIDE = 50
MAX_SIDE = 200
CLEARANCE = 20  # Increased clearance to ensure boxes don't touch
STEP = 10

# --- Generate Random Rectangles ---
random.seed(42)
sizes = [(random.randint(MIN_SIDE, MAX_SIDE), random.randint(MIN_SIDE, MAX_SIDE)) for _ in range(NUM_RECTS)]
rotated_sizes = [None] * NUM_RECTS  # Store whether each box is rotated

# --- Storage Density Calculation ---
def calculate_storage_density(packed_positions):
    if not packed_positions:
        return 0.0
    total_area = STORAGE_WIDTH * STORAGE_LENGTH
    occupied_area = sum(sizes[i][0] * sizes[i][1] for i in range(len(packed_positions)) if positions[i] is not None)
    return (occupied_area / total_area) * 100

# --- BFS Retrieval Path Function with Nearest Edge Selection ---
def find_retrieval_path(target_idx, current_positions, target_pos=None):
    if target_pos:
        tx, ty = target_pos
    else:
        tx, ty = current_positions[target_idx]
    tw, th = sizes[target_idx]

    # Calculate distance to all edges to find the nearest one
    distances = {
        'left': tx,
        'right': STORAGE_WIDTH - (tx + tw),
        'top': ty,
        'bottom': STORAGE_LENGTH - (ty + th)
    }
    
    # Select the nearest edge
    nearest_edge = min(distances, key=distances.get)
    
    # Determine target exit position based on nearest edge
    if nearest_edge == 'left':
        target_exit = (-tw, ty)  # Move completely off the left edge
    elif nearest_edge == 'right':
        target_exit = (STORAGE_WIDTH, ty)  # Move completely off the right edge
    elif nearest_edge == 'top':
        target_exit = (tx, -th)  # Move completely off the top edge
    else:  # bottom
        target_exit = (tx, STORAGE_LENGTH)  # Move completely off the bottom edge

    obstacles = []
    for j in range(NUM_RECTS):
        if j == target_idx or current_positions[j] is None:
            continue
        ox, oy = current_positions[j]
        ow, oh = sizes[j]
        obstacles.append((ox - CLEARANCE, oy - CLEARANCE, ow + 2 * CLEARANCE, oh + 2 * CLEARANCE))

    queue = deque([(tx, ty)])
    visited = {(tx, ty)}
    parent = {(tx, ty): None}
    end_pos = None
    
    while queue:
        x, y = queue.popleft()
        
        # Check if we've reached the target exit position or gone beyond storage boundaries
        if (nearest_edge == 'left' and x <= 0) or \
           (nearest_edge == 'right' and x + tw >= STORAGE_WIDTH) or \
           (nearest_edge == 'top' and y <= 0) or \
           (nearest_edge == 'bottom' and y + th >= STORAGE_LENGTH):
            end_pos = (x, y)
            break
        
        # Explore neighbors with priority towards the nearest edge
        directions = [(-STEP, 0), (STEP, 0), (0, -STEP), (0, STEP)]
        
        # Prioritize direction towards nearest edge
        if nearest_edge == 'left':
            directions = [(-STEP, 0), (0, -STEP), (0, STEP), (STEP, 0)]
        elif nearest_edge == 'right':
            directions = [(STEP, 0), (0, -STEP), (0, STEP), (-STEP, 0)]
        elif nearest_edge == 'top':
            directions = [(0, -STEP), (-STEP, 0), (STEP, 0), (0, STEP)]
        elif nearest_edge == 'bottom':
            directions = [(0, STEP), (-STEP, 0), (STEP, 0), (0, -STEP)]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            
            # Allow movement beyond storage boundaries for exit
            if (nx, ny) in visited:
                continue
            
            collision = False
            # Only check collision if still within storage area
            if 0 <= nx <= STORAGE_WIDTH - tw and 0 <= ny <= STORAGE_LENGTH - th:
                for ox, oy, ow, oh in obstacles:
                    if nx + tw > ox and nx < ox + ow and ny + th > oy and ny < oy + oh:
                        collision = True
                        break
            
            if not collision:
                visited.add((nx, ny))
                parent[(nx, ny)] = (x, y)
                queue.append((nx, ny))

    if end_pos:
        path = []
        current = end_pos
        while current is not None:
            path.append(current)
            current = parent[current]
        path.reverse()
        return path
    return None

# --- Find optimized packing path that avoids existing boxes ---
def find_packing_path(target_pos, box_size, existing_positions):
    target_x, target_y = target_pos
    box_w, box_h = box_size
    
    # All boxes must enter from top-left corner (0, STORAGE_LENGTH)
    start_x = 0
    start_y = STORAGE_LENGTH
    
    # Create obstacles from existing boxes
    obstacles = []
    for px, py in existing_positions:
        if (px, py) == target_pos:  # Skip the target position itself
            continue
        existing_idx = existing_positions.index((px, py))
        ew, eh = sizes[existing_idx]
        obstacles.append((px - CLEARANCE, py - CLEARANCE, ew + 2 * CLEARANCE, eh + 2 * CLEARANCE))
    
    # BFS pathfinding
    queue = deque([(start_x, start_y)])
    visited = {(start_x, start_y)}
    parent = {(start_x, start_y): None}
    
    while queue:
        x, y = queue.popleft()
        
        # Check if we've reached the target
        if x == target_x and y == target_y:
            path = []
            current = (x, y)
            while current is not None:
                path.append(current)
                current = parent[current]
            path.reverse()
            return path
        
        # Explore neighbors
        for dx, dy in [(-STEP, 0), (STEP, 0), (0, -STEP), (0, STEP)]:
            nx, ny = x + dx, y + dy
            
            if not (0 <= nx <= STORAGE_WIDTH - box_w and 0 <= ny <= STORAGE_LENGTH - box_h):
                continue
            if (nx, ny) in visited:
                continue
            
            collision = False
            for ox, oy, ow, oh in obstacles:
                if nx + box_w > ox and nx < ox + ow and ny + box_h > oy and ny < oy + oh:
                    collision = True
                    break
            
            if not collision:
                visited.add((nx, ny))
                parent[(nx, ny)] = (x, y)
                queue.append((nx, ny))
    
    return None  # No path found

# --- Check if a position would block future retrievals ---
def would_block_retrieval(new_pos, new_size, current_idx, existing_positions, retrieval_order):
    nx, ny = new_pos
    nw, nh = new_size
    
    # Create temporary positions including the new box
    temp_positions = existing_positions[:]
    temp_positions[current_idx] = (nx, ny)
    
    # Check if any future retrieval would be blocked
    for future_idx in retrieval_order:
        if temp_positions[future_idx] is None:
            continue
        
        path = find_retrieval_path(future_idx, temp_positions)
        if path is None:
            return True
    
    return False

# --- Smart Packing with Retrieval Consideration ---
positions = [None] * NUM_RECTS
packing_script = []
packed_positions = []

# Define retrieval order (random shuffle)
retrieval_order = list(range(NUM_RECTS))
random.shuffle(retrieval_order)

# Pack rectangles considering retrieval paths
for i, (w, h) in enumerate(sizes):
    placed = False
    best_orientation = None
    best_position = None
    
    # Try both orientations: original and rotated
    for width, height in [(w, h), (h, w)]:
        if placed:
            break
        for start_y in range(0, STORAGE_LENGTH - height, STEP):
            if placed:
                break
            for start_x in range(0, STORAGE_WIDTH - width, STEP):
                # Check if position has enough clearance from existing boxes
                collision = False
                for px, py in packed_positions:
                    existing_idx = packed_positions.index((px, py))
                    ew, eh = sizes[existing_idx]
                    if (start_x < px + ew + CLEARANCE and start_x + width + CLEARANCE > px and
                        start_y < py + eh + CLEARANCE and start_y + height + CLEARANCE > py):
                        collision = True
                        break
                
                if not collision:
                    # Check if this position would block future retrievals
                    if not would_block_retrieval((start_x, start_y), (width, height), i, positions, retrieval_order):
                        # Record the placement
                        best_orientation = (width, height)
                        best_position = (start_x, start_y)
                        placed = True
                        break
    
    # Place box using the best orientation found
    if best_orientation:
        w, h = best_orientation
        x, y = best_position
        positions[i] = (x, y)
        packed_positions.append((x, y))
        rotated_sizes[i] = (w, h) != sizes[i]  # True if rotated
        
        # Update sizes array to reflect rotation
        sizes[i] = (w, h)
        
        # Create optimized packing animation path from top-left corner (0, STORAGE_LENGTH)
        path = find_packing_path((x, y), (w, h), packed_positions[:-1])  # Exclude current position
        if not path:
            # Fallback to simple path from top-left corner
            path = []
            px, py = 0, STORAGE_LENGTH
            while px < x or py > y:
                if px < x:
                    px += STEP
                if py > y:
                    py -= STEP
                path.append((min(px, x), max(py, y)))
            path.append((x, y))
        
        for step in path:
            packing_script.append(("pack_move", i, step))
        packing_script.append(("pack_done", i, x, y))
    else:
        print(f"Warning: Could not place rectangle {i} without blocking retrieval paths")

# --- Animation Setup ---
fig, ax = plt.subplots(figsize=(12, 8))
ax.set_xlim(0, STORAGE_WIDTH)
ax.set_ylim(0, STORAGE_LENGTH)
ax.set_aspect('equal')
ax.set_title("2D Packing with Retrieval Path Planning - Non-Touching Boxes")

rect_patches = [Rectangle((0, 0), 0, 0, edgecolor='black', facecolor='lightblue', visible=False) for _ in range(NUM_RECTS)]
for patch in rect_patches:
    ax.add_patch(patch)

moving_patch = Rectangle((0, 0), 0, 0, edgecolor='red', facecolor='orange', lw=2, visible=False)
ax.add_patch(moving_patch)

store_line, = ax.plot([], [], 'r-', lw=2, label='Packing Path')
retrieve_line, = ax.plot([], [], 'b-', lw=3, label='Retrieval Path')
ax.legend()

# Add text for storage density display
density_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, fontsize=12, 
                       verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

current_positions = [None] * NUM_RECTS
script = []

# Add packing steps to script
script.extend(packing_script)

# Add retrieval steps
retrieval_paths = []
for idx in retrieval_order:
    current_positions[idx] = positions[idx]

for idx in retrieval_order:
    path = find_retrieval_path(idx, current_positions)
    retrieval_paths.append((idx, path))
    if path:
        for step in path:
            script.append(("retrieve", idx, step))
    script.append(("done", idx))
    current_positions[idx] = None  # Remove from current positions after retrieval

# --- Animation Function ---
def animate(frame):
    if frame >= len(script):
        return
    action = script[frame]

    if action[0] == "pack_move":
        idx, (x, y) = action[1], action[2]
        w, h = sizes[idx]
        moving_patch.set_xy((x, y))
        moving_patch.set_width(w)
        moving_patch.set_height(h)
        moving_patch.set_visible(True)
        moving_patch.set_facecolor('orange')
        moving_patch.set_edgecolor('red')
        
        # Show packing path
        path = [entry[2] for entry in script[:frame+1] if entry[0] == "pack_move" and entry[1] == idx]
        if path:
            px, py = zip(*path)
            store_line.set_data(px, py)

    elif action[0] == "pack_done":
        idx, x, y = action[1], action[2], action[3]
        w, h = sizes[idx]
        patch = rect_patches[idx]
        patch.set_xy((x, y))
        patch.set_width(w)
        patch.set_height(h)
        patch.set_visible(True)
        current_positions[idx] = (x, y)
        moving_patch.set_visible(False)
        store_line.set_data([], [])
        
        # Update storage density display
        packed_count = sum(1 for p in current_positions if p is not None)
        density = calculate_storage_density(packed_positions[:packed_count])
        density_text.set_text(f'Storage Density: {density:.1f}%\nBoxes Packed: {packed_count}/{NUM_RECTS}')

    elif action[0] == "retrieve":
        idx, (x, y) = action[1], action[2]
        w, h = sizes[idx]
        moving_patch.set_xy((x, y))
        moving_patch.set_width(w)
        moving_patch.set_height(h)
        moving_patch.set_visible(True)
        moving_patch.set_facecolor('yellow')
        moving_patch.set_edgecolor('blue')
        
        # Show retrieval path
        path = [entry[2] for entry in script[:frame+1] if entry[0] == "retrieve" and entry[1] == idx]
        if path:
            px, py = zip(*path)
            retrieve_line.set_data(px, py)

    elif action[0] == "done":
        idx = action[1]
        rect_patches[idx].set_visible(False)
        current_positions[idx] = None
        moving_patch.set_visible(False)
        retrieve_line.set_data([], [])
        
        # Update storage density display
        packed_count = sum(1 for p in current_positions if p is not None)
        remaining_packed = [i for i in range(NUM_RECTS) if current_positions[i] is not None]
        if remaining_packed:
            density = calculate_storage_density([current_positions[i] for i in remaining_packed])
        else:
            density = 0.0
        density_text.set_text(f'Storage Density: {density:.1f}%\nBoxes Remaining: {packed_count}/{NUM_RECTS}')

# --- Save animation as MP4 ---
print("Creating animation...")
print(f"Total frames: {len(script)}")
print(f"Boxes successfully packed: {sum(1 for p in positions if p is not None)}")

try:
    anim = FuncAnimation(fig, animate, frames=len(script), interval=100, repeat=False)
    writer = FFMpegWriter(fps=20, metadata=dict(artist='SmartPackingDemo'), bitrate=1800)
    anim.save("smart_packing_retrieval_2d.mp4", writer=writer)
    print("✅ Video saved as smart_packing_retrieval_2d.mp4")
except FileNotFoundError:
    print("❌ ffmpeg not found. Please install it and add to PATH.")
except Exception as e:
    print(f"❌ Error creating animation: {e}")

# --- Save detailed CSV files ---
with open("smart_rectangle_positions.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["index", "width", "height", "x", "y", "packed"])
    for i, (w, h) in enumerate(sizes):
        if positions[i] is not None:
            x, y = positions[i]
            writer.writerow([i, w, h, x, y, "Yes"])
        else:
            writer.writerow([i, w, h, "N/A", "N/A", "No"])

with open("smart_retrieval_paths.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["index", "step", "x", "y", "retrieval_order"])
    for order_idx, (idx, path) in enumerate(retrieval_paths):
        if path:
            for step_num, (x, y) in enumerate(path):
                writer.writerow([idx, step_num, x, y, order_idx])

print("✅ CSV files saved:")
print("  - smart_rectangle_positions.csv")
print("  - smart_retrieval_paths.csv")
print(f"✅ Successfully packed {sum(1 for p in positions if p is not None)} out of {NUM_RECTS} rectangles")
