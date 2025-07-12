import argparse
import requests
import json
import sys
import csv
import random

def make_packing_request(server_url, storage_width, storage_length, num_rects, min_side, max_side, clearance):
    """Make a POST request to the packing server"""
    
    payload = {
        "storage_width": storage_width,
        "storage_length": storage_length,
        "num_rects": num_rects,
        "min_side": min_side,
        "max_side": max_side,
        "clearance": clearance
    }
    
    try:
        response = requests.post(f"{server_url}/pack", json=payload)
        response.raise_for_status()
        return response.json()
    except requests.exceptions.RequestException as e:
        print(f"Error making request: {e}")
        return None

def save_result_to_file(result, filename):
    """Save the result to a JSON file"""
    try:
        with open(filename, 'w') as f:
            json.dump(result, f, indent=2)
        print(f"Result saved to {filename}")
    except Exception as e:
        print(f"Error saving file: {e}")

def create_packages_csv(placements, filename="packages.csv"):
    """Create packages.csv file from placements data"""
    try:
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["index", "width", "height", "x", "y", "packed"])
            for placement in placements:
                writer.writerow([
                    placement["id"],
                    placement["width"],
                    placement["height"],
                    placement["x"],
                    placement["y"],
                    "Yes"
                ])
        print(f"✅ Packages CSV saved: {filename}")
    except Exception as e:
        print(f"❌ Error saving packages CSV: {e}")

def create_retrieval_csv(placements, filename="retrieval.csv"):
    """Create retrieval.csv file with basic retrieval paths"""
    try:
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["index", "step", "x", "y", "retrieval_order"])
            
            # Create a simple retrieval order (random)
            retrieval_order = list(range(len(placements)))
            random.shuffle(retrieval_order)
            
            for order_idx, placement in enumerate(placements):
                # Create a simple path from current position to edge
                x, y = placement["x"], placement["y"]
                width, height = placement["width"], placement["height"]
                
                # Simple path: move to nearest edge
                steps = []
                
                # Move to left edge (simplified)
                for step in range(10):
                    new_x = x - step * 10
                    steps.append((new_x, y))
                
                # Add steps to CSV
                for step_num, (step_x, step_y) in enumerate(steps):
                    writer.writerow([placement["id"], step_num, step_x, step_y, order_idx])
                    
        print(f"✅ Retrieval CSV saved: {filename}")
    except Exception as e:
        print(f"❌ Error saving retrieval CSV: {e}")

def main():
    parser = argparse.ArgumentParser(description='Rectangle Packing Client')
    parser.add_argument('--server', default='http://localhost:3000', 
                       help='Server URL (default: http://localhost:3000)')
    parser.add_argument('--storage-width', type=int, default=1000,
                       help='Storage width (default: 1000)')
    parser.add_argument('--storage-length', type=int, default=2000,
                       help='Storage length (default: 2000)')
    parser.add_argument('--num-rects', type=int, default=50,
                       help='Number of rectangles (default: 50)')
    parser.add_argument('--min-side', type=int, default=50,
                       help='Minimum rectangle side length (default: 50)')
    parser.add_argument('--max-side', type=int, default=200,
                       help='Maximum rectangle side length (default: 200)')
    parser.add_argument('--clearance', type=int, default=20,
                       help='Clearance between rectangles (default: 20)')
    parser.add_argument('--output', default='packing_result.json',
                       help='Output file name (default: packing_result.json)')
    
    args = parser.parse_args()
    
    print("Making packing request with parameters:")
    print(f"  Storage: {args.storage_width} x {args.storage_length}")
    print(f"  Rectangles: {args.num_rects}")
    print(f"  Side range: {args.min_side} - {args.max_side}")
    print(f"  Clearance: {args.clearance}")
    print(f"  Server: {args.server}")
    
    # Make the request
    result = make_packing_request(
        args.server,
        args.storage_width,
        args.storage_length,
        args.num_rects,
        args.min_side,
        args.max_side,
        args.clearance
    )
    
    if result:
        print(f"\nPacking completed successfully!")
        print(f"Placed {len(result)} rectangles")
        print(f"Result type: {type(result)}")
        print(f"Result length: {len(result) if isinstance(result, list) else 'Not a list'}")
        
        # Save to JSON file
        save_result_to_file(result, args.output)
        
        # Create CSV files
        print("Creating packages.csv...")
        create_packages_csv(result, "packages.csv")
        print("Creating retrieval.csv...")
        create_retrieval_csv(result, "retrieval.csv")
        
        # Check if files were created
        import os
        if os.path.exists("packages.csv"):
            print("✅ packages.csv was created successfully")
        else:
            print("❌ packages.csv was NOT created")
            
        if os.path.exists("retrieval.csv"):
            print("✅ retrieval.csv was created successfully")
        else:
            print("❌ retrieval.csv was NOT created")
        
        # Print first few placements as example
        if result:
            print("\nFirst 3 placements:")
            for i, placement in enumerate(result[:3]):
                print(f"  Rectangle {placement['id']}: ({placement['x']}, {placement['y']}) "
                      f"{placement['width']}x{placement['height']}")
    else:
        print("Failed to get packing result")
        sys.exit(1)

if __name__ == "__main__":
    main()