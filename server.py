from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import json
import os
import random
import math

app = FastAPI()

# Allow CORS for local frontend development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Adjust for production!
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class PackingRequest(BaseModel):
    storage_width: int = 1000
    storage_length: int = 2000
    num_rects: int = 50
    min_side: int = 50
    max_side: int = 200
    clearance: int = 20

class Rectangle:
    def __init__(self, id, width, height):
        self.id = id
        self.width = width
        self.height = height
        self.x = 0
        self.y = 0
        self.placed = False

def generate_rectangles(num_rects, min_side, max_side):
    """Generate random rectangles"""
    rectangles = []
    for i in range(num_rects):
        width = random.randint(min_side, max_side)
        height = random.randint(min_side, max_side)
        rectangles.append(Rectangle(i, width, height))
    return rectangles

def can_place_rectangle(rect, x, y, storage_width, storage_length, clearance, placed_rectangles):
    """Check if rectangle can be placed at position (x, y)"""
    # Check if rectangle fits within storage bounds
    if x + rect.width + clearance > storage_width or y + rect.height + clearance > storage_length:
        return False
    
    # Check collision with other placed rectangles
    for placed_rect in placed_rectangles:
        if (x < placed_rect.x + placed_rect.width + clearance and 
            x + rect.width + clearance > placed_rect.x and
            y < placed_rect.y + placed_rect.height + clearance and 
            y + rect.height + clearance > placed_rect.y):
            return False
    
    return True

def pack_rectangles(rectangles, storage_width, storage_length, clearance):
    """Simple packing algorithm using bottom-left placement"""
    placed_rectangles = []
    
    for rect in rectangles:
        placed = False
        # Try to place rectangle at the bottom-left most position
        for y in range(0, storage_length, clearance):
            for x in range(0, storage_width, clearance):
                if can_place_rectangle(rect, x, y, storage_width, storage_length, clearance, placed_rectangles):
                    rect.x = x
                    rect.y = y
                    rect.placed = True
                    placed_rectangles.append(rect)
                    placed = True
                    break
            if placed:
                break
        
        if not placed:
            # If can't place, try without clearance
            for y in range(0, storage_length):
                for x in range(0, storage_width):
                    if can_place_rectangle(rect, x, y, storage_width, storage_length, 0, placed_rectangles):
                        rect.x = x
                        rect.y = y
                        rect.placed = True
                        placed_rectangles.append(rect)
                        placed = True
                        break
                if placed:
                    break

    return placed_rectangles

@app.post("/pack")
def pack_rectangles_endpoint(request: PackingRequest):
    try:
        # Generate rectangles
        rectangles = generate_rectangles(
            request.num_rects, 
            request.min_side, 
            request.max_side
        )
        
        # Pack rectangles
        placed_rectangles = pack_rectangles(
            rectangles, 
            request.storage_width, 
            request.storage_length, 
            request.clearance
        )
        
        # Convert to JSON format
        placements = []
        for rect in placed_rectangles:
            placements.append({
                "id": rect.id,
                "x": rect.x,
                "y": rect.y,
                "width": rect.width,
                "height": rect.height,
                "exit_edge": None,
                "path_length": None,
                "retrieval_path": None
            })
        
        result = placements
        
        return result
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/")
def root():
    return {"message": "Rectangle Packing API", "endpoints": ["POST /pack"]} 