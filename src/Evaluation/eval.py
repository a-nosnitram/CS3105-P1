# Evaluation script for A* and Best-First Search
# Metrics:
# 1. Solution quality (path cost)
# 2. Efficiency (number of nodes explored)
#
# Test categories:
# - eval1-5: Grid size variation (10, 15, 20, 25, 30)
# - eval6-10: Obstacle count variation (0, 1, 3, 6, 12)
# - eval11-15: Polygon vertex count variation (3, 4, 5, 6, 7)
# - eval16-20: Shape rigidity variation (rectangles to organic blobs)
# - eval21-25: Non-navigable pixel density variation (~1% to ~42%)

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import digitiser
import matplotlib.pyplot as plt
import numpy as np
from coord import Coord

# Add parent directory to path so we can import from src

# Using the same printMap function from P1main.py


def printMap(start, goal, size, obs, add=None):
    """Print visual representation of the grid (from P1main.py)
    Legend: '.' = free, 'O' = obstacle, 'S' = start, 'G' = goal, '*' = path/explored
    """
    if add is None:
        add = set()

    block = set()
    # collect the obstacles
    for o in obs:
        block.update(o.area)
    print()
    # print map
    print('  ', end='')
    for x in range(0, size):
        print(x % 10, end=" ")
    print()
    for y in range(0, size):
        print(y % 10, end=" ")
        for x in range(0, size):
            ch = '.'
            test = Coord(x, y)
            # decide what element to print on this cell
            if (test in block):
                ch = 'O'
            if (test in add):
                ch = '*'
            if (test == start):
                ch = 'S'
            if (test == goal):
                ch = 'G'
            print(ch, end=' ')
        print()
    print()


def load_problem(filename):
    """Load problem from file and return components"""
    with open(filename) as f:
        lines = f.readlines()

    size = int(lines[0])
    start = Coord(lines[1])
    goal = Coord(lines[2])

    obs = []
    for line in lines[3:]:
        split = line.strip().split(')(')
        vxs = []
        for o in split:
            c = Coord(o)
            vxs.append(c)
        poly = digitiser.drawPolygon(vxs)
        obs.append(poly)

    return size, start, goal, obs


def run_test_category(test_files, category_name, x_labels):
    """Run tests and collect results for a category"""
    results = {
        'AStar_cost': [],
        'BestF_cost': [],
        'Alt_cost': [],
        'AStar_nodes': [],
        'BestF_nodes': [],
        'Alt_nodes': [],
        'labels': x_labels
    }

    print(f"\n{'='*80}")
    print(f"  {category_name}")
    print(f"{'='*80}")

    for idx, test_file in enumerate(test_files):
        test_name = test_file.split('/')[-1].replace('.txt', '')
        print(
            f"\n[Test {idx+1}/{len(test_files)}] {test_name} ({x_labels[idx]})")
        print(f"{'-'*80}")

        size, start, goal, obs = load_problem(test_file)

        # Print problem details
        num_obstacles = len(obs)
        total_pixels = size * size
        obstacle_pixels = sum(len(o.area) for o in obs)
        coverage = (obstacle_pixels / total_pixels) * 100

        print(f"  Grid Size: {size}×{size} ({total_pixels} pixels)")
        print(f"  Start: {start}, Goal: {goal}")
        print(
            f"  Obstacles: {num_obstacles}, Coverage: {obstacle_pixels}/{total_pixels} ({coverage:.1f}%)")

        # print map
        print(f"     Legend: S=Start, G=Goal, O=Obstacle, .=Free")
        printMap(start, goal, size, obs)

    # Run A*
        from AStar import AStar
        import time
        start_time = time.time()
        astar = AStar(size, start, goal, obs, verbose=False)
        astar_cost, astar_nodes = astar.search()
        astar_time = time.time() - start_time

        results['AStar_cost'].append(
            astar_cost if astar_cost is not None else 0)
        results['AStar_nodes'].append(astar_nodes)

        if astar_cost is not None:
            print(f"     Path Found :)")
            print(f"     - Path Cost: {astar_cost:.2f}")
            print(f"     - Nodes Explored: {astar_nodes}")
            print(f"     - Time: {astar_time:.4f}s")
        else:
            print(f"     No Path Found :(")
            print(f"     - Nodes Explored: {astar_nodes}")
            print(f"     - Time: {astar_time:.4f}s")

    # Run BestF
        from BestF import BestF
        start_time = time.time()
        bestf = BestF(size, start, goal, obs, verbose=False)
        bestf_cost, bestf_nodes = bestf.search()
        bestf_time = time.time() - start_time

        results['BestF_cost'].append(
            bestf_cost if bestf_cost is not None else 0)
        results['BestF_nodes'].append(bestf_nodes)

        if bestf_cost is not None:
            print(f"     Path Found!")
            print(f"     - Path Cost: {bestf_cost:.2f}")
            print(f"     - Nodes Explored: {bestf_nodes}")
            print(f"     - Time: {bestf_time:.4f}s")
        else:
            print(f"     No Path Found")
            print(f"     - Nodes Explored: {bestf_nodes}")
            print(f"     - Time: {bestf_time:.4f}s")

        # Run Alt (RBFS)
        from Alt import Alt
        start_time = time.time()
        alt = Alt(size, start, goal, obs, verbose=False)
        alt_cost, alt_nodes = alt.search()
        alt_time = time.time() - start_time

        results['Alt_cost'].append(
            alt_cost if alt_cost is not None else 0)
        results['Alt_nodes'].append(alt_nodes)

        if alt_cost is not None:
            print(f"     Path Found!")
            print(f"     - Path Cost: {alt_cost:.2f} (Alt)")
            print(f"     - Nodes Explored: {alt_nodes}")
            print(f"     - Time: {alt_time:.4f}s")
        else:
            print(f"     No Path Found (Alt)")
            print(f"     - Nodes Explored: {alt_nodes}")
            print(f"     - Time: {alt_time:.4f}s")

        # Comparison
        print(f"\nComparison:")
        times = {
            'A*': astar_time,
            'Best-First': bestf_time,
            'Alt': alt_time
        }
        fastest_name = min(times, key=times.get)
        slowest_time = max(times.values())
        fastest_time = times[fastest_name]
        if fastest_time > 0 and slowest_time / fastest_time > 1.1:
            print(f"     - Fastest: {fastest_name} ({fastest_time:.4f}s)")
            for name, t in times.items():
                if name != fastest_name:
                    print(f"       · {name}: {t/fastest_time:.2f}x slower")

    return results


def plot_comparison(results, category_name, x_label, filename):
    """Create comparison plots for a test category with multiple visualization types"""
    # Create a 2x2 grid of plots
    fig = plt.figure(figsize=(16, 12))
    gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
    fig.suptitle(f'{category_name} - Algorithm Comparison',
                 fontsize=18, fontweight='bold', y=0.98)

    x = np.arange(len(results['labels']))
    width = 0.25

    # Plot 1: Path Cost - Bar Chart
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.bar(x - width, results['AStar_cost'],
            width, label='A*', color='#F28235', alpha=0.8)
    ax1.bar(x, results['BestF_cost'], width,
            label='Best-First', color='#9ECF34', alpha=0.8)
    ax1.bar(x + width, results['Alt_cost'], width,
            label='Alt (RBFS)', color='#4BA3F2', alpha=0.8)
    ax1.set_xlabel(x_label, fontsize=11)
    ax1.set_ylabel('Path Cost', fontsize=11)
    ax1.set_title('Solution Quality - Bar Chart',
                  fontsize=12, fontweight='bold')
    ax1.set_xticks(x)
    ax1.set_xticklabels(results['labels'], rotation=15, ha='right')
    ax1.legend()
    ax1.grid(axis='y', alpha=0.3)

    # Plot 2: Nodes Explored - Bar Chart
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.bar(x - width, results['AStar_nodes'],
            width, label='A*', color='#F28235', alpha=0.8)
    ax2.bar(x, results['BestF_nodes'], width,
            label='Best-First', color='#9ECF34', alpha=0.8)
    ax2.bar(x + width, results['Alt_nodes'], width,
            label='Alt (RBFS)', color='#4BA3F2', alpha=0.8)
    ax2.set_xlabel(x_label, fontsize=11)
    ax2.set_ylabel('Nodes Explored (log scale)', fontsize=11)
    ax2.set_title('Efficiency - Bar Chart (Log Scale)', fontsize=12, fontweight='bold')
    ax2.set_xticks(x)
    ax2.set_xticklabels(results['labels'], rotation=15, ha='right')
    ax2.set_yscale('log')  # Use logarithmic scale for better visibility
    ax2.legend()
    ax2.grid(axis='y', alpha=0.3, which='both', linestyle=':')

    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Saved plot: {filename}")
    plt.close()


def main():
    """Run all evaluation tests"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    eval_dir = os.path.join(script_dir, "eval_tests")
    output_dir = os.path.join(script_dir, "results")

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)

    # Store all results for overall comparison
    all_results = []

    # Category 1: Grid Size Variation (eval1-5)
    print("\n=== Category 1: Grid Size Variation ===")
    test_files = [f"{eval_dir}/eval{i}_grid_size.txt" for i in range(1, 6)]
    labels = ['10x10', '15x15', '20x20', '25x25', '30x30']
    results1 = run_test_category(test_files, "Grid Size Variation", labels)
    plot_comparison(results1, "Grid Size Variation",
                    "Grid Size", f"{output_dir}/01_grid_size.png")
    all_results.append(('Grid Size', results1))

    # Category 2: Obstacle Count Variation (eval6-10)
    print("\n=== Category 2: Obstacle Count Variation ===")
    test_files = [f"{eval_dir}/eval{i}_obs_num.txt" for i in range(6, 11)]
    labels = ['0', '1', '3', '6', '12']
    results2 = run_test_category(
        test_files, "Obstacle Count Variation", labels)
    plot_comparison(results2, "Obstacle Count Variation",
                    "Number of Obstacles", f"{output_dir}/02_obstacle_count.png")
    all_results.append(('Obstacle Count', results2))

    # Category 3: Vertex Count Variation (eval11-15)
    print("\n=== Category 3: Polygon Vertex Count ===")
    test_files = [f"{eval_dir}/eval{i}_edge_num.txt" for i in range(11, 16)]
    labels = ['3', '4', '5', '6', '7']
    results3 = run_test_category(test_files, "Polygon Vertex Count", labels)
    plot_comparison(results3, "Polygon Vertex Count",
                    "Vertices per Polygon", f"{output_dir}/03_vertex_count.png")
    all_results.append(('Vertex Count', results3))

    # Category 4: Shape Rigidity (eval16-20)
    print("\n=== Category 4: Shape Rigidity ===")
    test_files = [f"{eval_dir}/eval{i}_shape.txt" for i in range(16, 21)]
    labels = ['Rectangle', 'Pentagon', 'Hexagon', 'Irregular', 'Organic']
    results4 = run_test_category(test_files, "Shape Rigidity", labels)
    plot_comparison(results4, "Shape Rigidity", "Shape Complexity",
                    f"{output_dir}/04_shape_rigidity.png")
    all_results.append(('Shape Rigidity', results4))

    # Category 5: Obstacle Density (eval21-25)
    print("\n=== Category 5: Obstacle Density ===")
    test_files = [f"{eval_dir}/eval{i}_ratio.txt" for i in range(21, 26)]
    labels = ['~1%', '~5%', '~13%', '~25%', '~42%']
    results5 = run_test_category(test_files, "Obstacle Density", labels)
    plot_comparison(results5, "Obstacle Density", "Non-navigable Pixel Coverage",
                    f"{output_dir}/05_obstacle_density.png")
    all_results.append(('Obstacle Density', results5))

    # Generate overall summary plots as well
    create_summary_plot(output_dir, all_results)


def create_summary_plot(output_dir, all_results):
    """Create comprehensive summary plots showing overall performance across all tests"""

    # Calculate aggregate statistics
    categories = []
    avg_astar_cost = []
    avg_bestf_cost = []
    avg_alt_cost = []
    std_astar_cost = []
    std_bestf_cost = []
    std_alt_cost = []
    avg_astar_nodes = []
    avg_bestf_nodes = []
    avg_alt_nodes = []
    std_astar_nodes = []
    std_bestf_nodes = []
    std_alt_nodes = []
    total_astar_cost = []
    total_bestf_cost = []
    total_alt_cost = []
    total_astar_nodes = []
    total_bestf_nodes = []
    total_alt_nodes = []

    for cat_name, results in all_results:
        categories.append(cat_name)
        # Means
        avg_astar_cost.append(np.mean(results['AStar_cost']))
        avg_bestf_cost.append(np.mean(results['BestF_cost']))
        avg_alt_cost.append(np.mean(results['Alt_cost']))
        avg_astar_nodes.append(np.mean(results['AStar_nodes']))
        avg_bestf_nodes.append(np.mean(results['BestF_nodes']))
        avg_alt_nodes.append(np.mean(results['Alt_nodes']))
        # Standard deviations (error bars)
        std_astar_cost.append(np.std(results['AStar_cost']))
        std_bestf_cost.append(np.std(results['BestF_cost']))
        std_alt_cost.append(np.std(results['Alt_cost']))
        std_astar_nodes.append(np.std(results['AStar_nodes']))
        std_bestf_nodes.append(np.std(results['BestF_nodes']))
        std_alt_nodes.append(np.std(results['Alt_nodes']))
        # Totals
        total_astar_cost.append(np.sum(results['AStar_cost']))
        total_bestf_cost.append(np.sum(results['BestF_cost']))
        total_alt_cost.append(np.sum(results['Alt_cost']))
        total_astar_nodes.append(np.sum(results['AStar_nodes']))
        total_bestf_nodes.append(np.sum(results['BestF_nodes']))
        total_alt_nodes.append(np.sum(results['Alt_nodes']))

    # Create overall comparison figure
    fig = plt.figure(figsize=(18, 12))
    gs = fig.add_gridspec(2, 2, hspace=0.35, wspace=0.35)
    # fig.suptitle('Overall Performance Comparison: A* vs Best-First Search (All 25 Tests)',
    #  fontsize=20, fontweight='bold', y=0.98)

    x = np.arange(len(categories))
    width = 0.35

    # Plot 1: Average Path Cost by Category
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.bar(x - width, avg_astar_cost, width,
        yerr=std_astar_cost, capsize=5,
        label='A*', color='#F28235', alpha=0.8, ecolor='#9a9a9a')
    ax1.bar(x, avg_bestf_cost, width,
        yerr=std_bestf_cost, capsize=5,
        label='Best-First', color='#9ECF34', alpha=0.8, ecolor='#9a9a9a')
    ax1.bar(x + width, avg_alt_cost, width,
        yerr=std_alt_cost, capsize=5,
        label='Alt (RBFS)', color='#4BA3F2', alpha=0.8, ecolor='#9a9a9a')
    ax1.set_ylabel('Average Path Cost', fontsize=11)
    ax1.set_title('Figure 3: Average Solution Quality by Category',
                  fontsize=12, fontweight='bold')
    ax1.set_xticks(x)
    ax1.set_xticklabels(categories, rotation=20, ha='right', fontsize=9)
    ax1.legend()
    ax1.grid(axis='y', alpha=0.3)

    # Plot 2: Average Nodes Explored by Category
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.bar(x - width, avg_astar_nodes, width,
        yerr=std_astar_nodes, capsize=5,
        label='A*', color='#F28235', alpha=0.8, ecolor='#9a9a9a')
    ax2.bar(x, avg_bestf_nodes, width,
        yerr=std_bestf_nodes, capsize=5,
        label='Best-First', color='#9ECF34', alpha=0.8, ecolor='#9a9a9a')
    ax2.bar(x + width, avg_alt_nodes, width,
        yerr=std_alt_nodes, capsize=5,
        label='Alt (RBFS)', color='#4BA3F2', alpha=0.8, ecolor='#9a9a9a')
    ax2.set_ylabel('Average Nodes Explored (log scale)', fontsize=11)
    ax2.set_title('Figure 4: Average Efficiency by Category (Log Scale)',
                  fontsize=12, fontweight='bold')
    ax2.set_xticks(x)
    ax2.set_xticklabels(categories, rotation=20, ha='right', fontsize=9)
    ax2.set_yscale('log')  # Use logarithmic scale for better visibility
    ax2.legend()
    ax2.grid(axis='y', alpha=0.3, which='both', linestyle=':')

    # Plot 3: Pie Chart - Total Nodes Explored
    # ax3 = fig.add_subplot(gs[1, :])
    # total_all_astar = sum(total_astar_nodes)
    # total_all_bestf = sum(total_bestf_nodes)
    # total_all_alt = sum(total_alt_nodes)
    # ax3.pie([total_all_astar, total_all_bestf, total_all_alt], labels=['A*', 'Best-First', 'Alt (RBFS)'],
    #         colors=['#F28235', '#9ECF34', '#4BA3F2'], autopct='%1.1f%%', startangle=90,
    #         textprops={'fontsize': 12, 'fontweight': 'bold'})
    # ax3.set_title(
    #     f'Figure 5: Total Nodes Explored\nAcross All Tests\n(Total: {total_all_astar + total_all_bestf + total_all_alt:,})',
    #     fontsize=12, fontweight='bold')

    plt.savefig(f"{output_dir}/00_overall_summary.png",
                dpi=300, bbox_inches='tight')
    print(f"Saved plot: {output_dir}/00_overall_summary.png")
    plt.close()

    # Create a second summary with line charts showing all tests
    create_all_tests_plot(output_dir, all_results)


def create_all_tests_plot(output_dir, all_results):
    """Create a comprehensive plot showing results for all 25 tests"""
    fig = plt.figure(figsize=(20, 10))
    gs = fig.add_gridspec(2, 2, hspace=0.3, wspace=0.25)
    # fig.suptitle('Performance Over 25 Tests Across 5 Categories',
    #              fontsize=20, fontweight='bold', y=0.98)

    # Flatten all results and build descriptive labels
    all_labels = []
    all_astar_cost = []
    all_bestf_cost = []
    all_alt_cost = []
    all_astar_nodes = []
    all_bestf_nodes = []
    all_alt_nodes = []
    category_boundaries = [0]

    for cat_name, results in all_results:
        for i, label in enumerate(results['labels']):
            # Build a more descriptive label for each test
            if cat_name == 'Grid Size':
                test_label = f"{label}"
            elif cat_name == 'Obstacle Count':
                test_label = f"{label}"
            elif cat_name == 'Vertex Count':
                test_label = f"{label}"
            elif cat_name == 'Shape Rigidity':
                test_label = f"{label}"
            elif cat_name == 'Obstacle Density':
                test_label = f"{label}"
            else:
                test_label = f"{cat_name}: {label}"
            all_labels.append(test_label)
            all_astar_cost.append(results['AStar_cost'][i])
            all_bestf_cost.append(results['BestF_cost'][i])
            all_alt_cost.append(results['Alt_cost'][i])
            all_astar_nodes.append(results['AStar_nodes'][i])
            all_bestf_nodes.append(results['BestF_nodes'][i])
            all_alt_nodes.append(results['Alt_nodes'][i])
        category_boundaries.append(len(all_labels))

    x = np.arange(len(all_labels))

    # Plot 1: All Path Costs
    ax1 = fig.add_subplot(gs[0, :])
    # Plot Best-First first (background)
    ax1.plot(x, all_bestf_cost, 's-', label='Best-First',
             color='#9ECF34', linewidth=2.5, markersize=7, alpha=0.85)
    # Plot A* with hollow markers
    ax1.plot(x, all_astar_cost, 'o-', label='A*',
             color='#F28235', linewidth=2.5, markersize=9, alpha=0.9,
             markerfacecolor='none', markeredgewidth=2)
    # Plot Alt with different marker
    ax1.plot(x, all_alt_cost, 'D-', label='Alt (RBFS)',
             color='#4BA3F2', linewidth=2, markersize=6, alpha=0.9)

    # Add vertical lines to separate categories
    for boundary in category_boundaries[1:-1]:
        ax1.axvline(x=boundary-0.5, color='gray',
                    linestyle='--', linewidth=1, alpha=0.5)

    # Add category labels
    for i, (cat_name, _) in enumerate(all_results):
        mid_point = (category_boundaries[i] + category_boundaries[i+1]) / 2
        ax1.text(mid_point-0.5, ax1.get_ylim()[1] * 0.95, cat_name,
                 ha='center', va='top', fontsize=10, fontweight='bold',
                 bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.3))

    # ax1.set_xlabel('Test', fontsize=13)
    ax1.set_ylabel('Path Cost', fontsize=13)
    ax1.set_title('Figure 1: Solution Quality Across All 25 Tests',
                  fontsize=14, fontweight='bold')
    ax1.set_xticks(x)
    ax1.set_xticklabels(all_labels, fontsize=8, rotation=45, ha='right')
    ax1.legend(fontsize=12)
    ax1.grid(True, alpha=0.3)

    # Plot 2: All Nodes Explored
    ax2 = fig.add_subplot(gs[1, :])
    ax2.plot(x, all_astar_nodes, 'o-', label='A*',
             color='#F28235', linewidth=2, markersize=6, alpha=0.8)
    ax2.plot(x, all_bestf_nodes, 's-', label='Best-First',
             color='#9ECF34', linewidth=2, markersize=6, alpha=0.8)
    ax2.plot(x, all_alt_nodes, '^-', label='Alt (RBFS)',
             color='#4BA3F2', linewidth=2, markersize=6, alpha=0.8)

    # Add vertical lines to separate categories
    for boundary in category_boundaries[1:-1]:
        ax2.axvline(x=boundary-0.5, color='gray',
                    linestyle='--', linewidth=1, alpha=0.5)

    for i, (cat_name, _) in enumerate(all_results):
        mid_point = (category_boundaries[i] + category_boundaries[i+1]) / 2
        ax2.text(mid_point-0.5, ax2.get_ylim()[1] * 0.95, cat_name,
                 ha='center', va='top', fontsize=10, fontweight='bold',
                 bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.3))

    # ax2.set_xlabel('Test', fontsize=13)
    ax2.set_ylabel('Nodes Explored (log scale)', fontsize=13)
    ax2.set_title('Figure 2: Nodes Explored Across All 25 Tests (Log Scale)',
                  fontsize=14, fontweight='bold')
    ax2.set_xticks(x)
    ax2.set_xticklabels(all_labels, fontsize=8, rotation=45, ha='right')
    ax2.set_yscale('log')  # Use logarithmic scale for better visibility
    ax2.legend(fontsize=12)
    ax2.grid(True, alpha=0.3, which='both', linestyle=':')

    # Add coloured background to each section
    for i in range(len(category_boundaries)-1):
        start = category_boundaries[i]
        end = category_boundaries[i+1]-1
        color = '#f0f8ff' if i % 2 == 0 else '#faebd7'
        ax1.axvspan(start-0.5, end+0.5, color=color, alpha=0.3)
        ax2.axvspan(start-0.5, end+0.5, color=color, alpha=0.3)

    plt.savefig(f"{output_dir}/00_all_tests_detailed.png",
                dpi=300, bbox_inches='tight')
    print(f"Saved plot: {output_dir}/00_all_tests_detailed.png")
    plt.close()


if __name__ == "__main__":
    main()
