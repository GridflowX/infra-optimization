from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import json
from runner import run_algorithm

app = FastAPI()

class InputParams(BaseModel):
    alpha: float
    beta: float

@app.post("/input")
def run(params: InputParams):
    try:
        output = run_algorithm(params.alpha, params.beta)
        return {"status": "success", "output": output}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/output")
def get_result():
    try:
        # First try to read json_output.json
        with open("json_output.json", "r") as f:
            return json.load(f)
    except FileNotFoundError:
        try:
            # Fallback to graph_output.json if json_output.json doesn't exist
            with open("graph_output.json", "r") as f:
                return json.load(f)
        except FileNotFoundError:
            raise HTTPException(status_code=404, detail="No result found")
