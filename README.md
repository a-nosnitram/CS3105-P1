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
- `eval11-15`: Polygon vertex count variation (3, 4, 5, 6, 7-9)
- `eval16-20`: Shape rigidity variation (rectangles to organic blobs)
- `eval21-25`: Non-navigable pixel density variation (~1% to ~42%)

## Analytics Produced 

The evaluation script compares the performance of A* and Best-First Search algorithms across 25 test cases, grouped into five categories. For each category and overall, the following analytics are generated and saved as `.png` files in `src/Evaluation/results/`:

- **Per Category Comparison Plots**:
  - Path cost (solution quality) bar and line charts
  - Nodes explored (efficiency) bar and line charts
  - Efficiency ratio (BestF/A*) bar chart
  - Solution quality difference (BestF - A*) bar chart
  - Scatter plot: cost vs nodes explored

- **Overall Summary Plots**:
  - Average path cost and nodes explored by category
  - Pie chart: total nodes explored by each algorithm
  - Efficiency ratio by category
  - Solution quality difference by category
  - Scatter plot: cost vs nodes explored (all tests)
  - Heatmap: average path cost per category
  - Pie chart: number of tests where A* or BestF found better solutions, or tied
  - Statistics summary table (averages, totals, wins)

- **Detailed All-Tests Plot**:
  - Path cost and nodes explored for all 25 tests
  - Efficiency improvement (% fewer nodes explored by A*)
  - Solution quality difference for each test