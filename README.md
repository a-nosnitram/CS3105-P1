# How to run evaluation 
In the root directory of the porject (where this README file is located), run the following commands:

```
chmod -x run_eval.sh
bash run_eval.sh
```

The `run_eval.sh` script will run `eval.py` located in `src/Evaluation`. The Python script will generate `.png` analytics graphs in the `src/Evaluation/results/` directory, based on the results from the 25 test cases in `src/Evaluation/eval_tests/`. 

## Test Categories 
- `eval1-5`: Grid size variation (10, 15, 20, 25, 30)
- `eval6-10`: Obstacle count variation (0, 1, 3, 6, 10)
- `eval11-15`: Polygon vertex count variation (3, 4, 5, 6, 7)
- `eval16-20`: Shape rigidity variation (rectangles to organic blobs)
- `eval21-25`: Non-navigable pixel density variation (~1% to ~42%)
