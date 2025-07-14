# packing retrieval optimization

This project implements an **automated 2D warehouse packing and retrieval system**. It is designed to efficiently pack rectangular objects into a fixed warehouse space while ensuring that every object can be retrieved via a collision-free path to at least one edge. The system combines geometric packing algorithms with motion planning to optimize both space utilization and retrievability.

---

## Features

- **Packing Algorithm**: Packs 2D rectangles of varying sizes into a warehouse using a modified skyline or bottom-left heuristic, with optional rotation and clearance enforcement.
- **Retrieval Path Planning**: Ensures every object has a collision-free, shortest path to a warehouse edge using probabilistic roadmaps and A* search.
- **Visualization**: Generates visual outputs and animations of the packing and retrieval process.
- **Data Export**: Outputs results as JSON and CSV files for further analysis or integration.

---

## Key Components

- `server.py`: FastAPI server providing a `/pack` endpoint for packing requests. Generates `packages.csv` and `retrieval.csv` with packing and retrieval data.
- `main.py`: Command-line client to interact with the server, save results, and generate CSV files.
- `packing_optimization_matplotlib.py`: Advanced script for large-scale packing with visualization and animation (requires `matplotlib` and `ffmpeg`).
- `context.md`: Detailed technical and conceptual documentation of the algorithms and system design.

---

## How It Works

1. **Input**: Define or randomly generate rectangle dimensions.
2. **Packing**: Rectangles are packed into the warehouse, maximizing space utilization and maintaining a specified clearance.
3. **Retrieval Planning**: For each packed rectangle, the system computes a collision-free path to the nearest edge.
4. **Output**: Results are saved as:
   - `packages.csv`: Packed rectangle positions and sizes.
   - `retrieval.csv`: Step-by-step retrieval paths.
   - (Optional) Visualizations and animations.

---

## Usage

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Run the FastAPI Server

```bash
uvicorn server:app --reload --port 3000
```

### 3. Use the Command-Line Client

```bash
python main.py --server http://localhost:3000 --storage-width 1000 --storage-length 2000 --num-rects 50 --min-side 50 --max-side 200 --clearance 20 --output packing_result.json
```

This will:
- Send a packing request to the server.
- Save the result as `packing_result.json`.
- Generate `packages.csv` and `retrieval.csv`.

### 4. Advanced Visualization (Optional)

To create an animation and detailed CSVs for large-scale packing:

```bash
python packing_optimization_matplotlib.py
```

This will generate:
- `smart_packing_retrieval_2d.mp4` (animation)
- `smart_rectangle_positions.csv`
- `smart_retrieval_paths.csv`

---

## File Descriptions

- **`server.py`**: API server for packing and retrieval.
- **`main.py`**: CLI client for interacting with the server and generating outputs.
- **`packing_optimization_matplotlib.py`**: Visualization and animation for large-scale scenarios.
- **`packages.csv`**: Packed rectangle data.
- **`retrieval.csv`**: Retrieval path data.
- **`requirements.txt`**: Python dependencies.
- **`context.md`**: In-depth technical documentation.

---

## Algorithms Used

| Component          | Technique Used                 | Purpose                          |
|--------------------|-------------------------------|----------------------------------|
| Packing            | Skyline/Bottom-Left Heuristic | Maximize space utilization       |
| Collision Check    | Rectangle Intersection        | Ensure clearance                 |
| Roadmap            | Probabilistic Roadmap (PRM)   | Navigable space for retrieval    |
| Path Finding       | A* Search                     | Shortest retrieval paths         |
| Optimization       | KD-trees, Heuristics          | Computational efficiency         |

---

## Applications

- Warehouse automation
- Manufacturing floor planning
- Logistics and container loading
- Public infrastructure (e.g., emergency exit planning)

---

## License

This project is licensed under the [Apache License 2.0](LICENSE).

---

## Acknowledgements

- Developed for research and practical deployment in warehouse and logistics automation.
- See `context.md` for detailed background, limitations, and future work.

---

## Contact

For questions or contributions, please open an issue or pull request on the repository. 
