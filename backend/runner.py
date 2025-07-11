import subprocess
import json
import os

def run_algorithm(alpha, beta):
    result = subprocess.run(
        [
            "docker", "run", "--rm",
            "-v", f"{os.getcwd()}:/app",  # Mount current directory to /app in container
            "-e", f"ALPHA={alpha}",
            "-e", f"BETA={beta}",
            "guideway_optimization"  # Change this to your built Docker image name
        ],
        capture_output=True,
        text=True
    )

    if result.returncode != 0:
        raise Exception(f"Docker failed: {result.stderr.strip()}")
    
    # After successful Docker run, read the generated JSON output
    json_output_path = "json_output.json"
    if os.path.exists(json_output_path):
        with open(json_output_path, "r") as f:
            return json.load(f)
    else:
        # If json_output.json doesn't exist, try graph_output.json as fallback
        graph_output_path = "graph_output.json"
        if os.path.exists(graph_output_path):
            with open(graph_output_path, "r") as f:
                return json.load(f)
        else:
            raise Exception("No JSON output found after Docker execution")
