# Evaluation script for A* and Best-First Search
# Metrics:
# 1. Solution quality (path cost)
# 2. Efficiency (number of nodes explored)
#
# Test categories:
# - eval1-5: Grid size variation (10, 15, 20, 25, 30)
# - eval6-10: Obstacle count variation (0, 1, 3, 6, 12)
# - eval11-15: Polygon vertex count variation (3, 4, 5, 6, 7-9)
# - eval16-20: Shape rigidity variation (rectangles to organic blobs)
# - eval21-25: Non-navigable pixel density variation (~1% to ~42%)

import digitiser
from coord import Coord
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# Add parent directory to path so we can import from src
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


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


def run_algorithm(algo_name, size, start, goal, obs):
    """Run algorithm and return (path_cost, nodes_explored)"""
    if algo_name == "AStar":
        from AStar import AStar
        algo = AStar(size, start, goal, obs, verbose=False)
    elif algo_name == "BestF":
        from BestF import BestF
        algo = BestF(size, start, goal, obs, verbose=False)
    else:
        return None, 0

    # Capture explored count
    start_node_count = len(getattr(algo, 'explored', set()))
    path_cost = algo.search()

    # Count explored nodes (approximation based on frontier operations)
    # We'll need to modify the algorithms slightly to track this properly
    # For now, we'll run and count

    return path_cost, start_node_count


def run_with_tracking(algo_name, size, start, goal, obs):
    """Run algorithm with proper tracking of explored nodes"""
    if algo_name == "AStar":
        from AStar import AStar
        algo = AStar(size, start, goal, obs, verbose=False)
    elif algo_name == "BestF":
        from BestF import BestF
        algo = BestF(size, start, goal, obs, verbose=False)
    else:
        return None, 0

    # Monkey-patch to track explored nodes
    from queue import PriorityQueue
    from node import Node
    import itertools
    from digitiser import DrawLine

    explored = set()
    nodes_explored = 0

    start_node = Node(algo.state_space.start)
    start_node.cost = 0
    start_node.her = algo.heuristic(algo.state_space.start)
    if algo_name == "AStar":
        start_node.total_cost = start_node.cost + start_node.her
    else:  # BestF
        start_node.total_cost = start_node.her

    algo.frontier.put((start_node.total_cost, next(algo._counter), start_node))

    while not algo.frontier.empty():
        current_node = algo.frontier.get()[2]
        explored.add((current_node.coord.x, current_node.coord.y))
        nodes_explored += 1

        if current_node.coord == algo.state_space.goal:
            return float(current_node.cost), nodes_explored

        neighbors = algo.state_space.get_neighbors(current_node)
        for neighbor in neighbors:
            neighbor_node = Node(neighbor, current_node)
            line = DrawLine(current_node.coord, neighbor)
            distance = line.length
            neighbor_node.cost = current_node.cost + distance
            neighbor_node.her = algo.heuristic(neighbor)

            if algo_name == "AStar":
                neighbor_node.total_cost = neighbor_node.cost + neighbor_node.her
            else:  # BestF
                neighbor_node.total_cost = neighbor_node.her

            coord_key = (neighbor_node.coord.x, neighbor_node.coord.y)
            if coord_key not in explored:
                algo.frontier.put(
                    (neighbor_node.total_cost, next(algo._counter), neighbor_node))

    return None, nodes_explored


def calculate_obstacle_coverage(size, obs):
    """Calculate percentage of grid covered by obstacles"""
    total_pixels = size * size
    obstacle_pixels = 0
    for o in obs:
        obstacle_pixels += len(o.area)
    return (obstacle_pixels / total_pixels) * 100


def run_test_category(test_files, category_name, x_labels):
    """Run tests and collect results for a category"""
    results = {
        'AStar_cost': [],
        'BestF_cost': [],
        'AStar_nodes': [],
        'BestF_nodes': [],
        'labels': x_labels
    }

    for test_file in test_files:
        print(f"Running {test_file}...")
        size, start, goal, obs = load_problem(test_file)

        # Run A*
        astar_cost, astar_nodes = run_with_tracking(
            "AStar", size, start, goal, obs)
        results['AStar_cost'].append(
            astar_cost if astar_cost is not None else 0)
        results['AStar_nodes'].append(astar_nodes)

        # Run BestF
        bestf_cost, bestf_nodes = run_with_tracking(
            "BestF", size, start, goal, obs)
        results['BestF_cost'].append(
            bestf_cost if bestf_cost is not None else 0)
        results['BestF_nodes'].append(bestf_nodes)

    return results


def plot_comparison(results, category_name, x_label, filename):
    """Create comparison plots for a test category with multiple visualization types"""
    # Create a 2x2 grid of plots
    fig = plt.figure(figsize=(16, 12))
    gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
    fig.suptitle(f'{category_name} - Algorithm Comparison',
                 fontsize=18, fontweight='bold', y=0.98)

    x = np.arange(len(results['labels']))
    width = 0.35

    # Plot 1: Path Cost - Bar Chart
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.bar(x - width/2, results['AStar_cost'],
            width, label='A*', color='#2ecc71', alpha=0.8)
    ax1.bar(x + width/2, results['BestF_cost'], width,
            label='Best-First', color='#e74c3c', alpha=0.8)
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
    ax2.bar(x - width/2, results['AStar_nodes'],
            width, label='A*', color='#2ecc71', alpha=0.8)
    ax2.bar(x + width/2, results['BestF_nodes'], width,
            label='Best-First', color='#e74c3c', alpha=0.8)
    ax2.set_xlabel(x_label, fontsize=11)
    ax2.set_ylabel('Nodes Explored', fontsize=11)
    ax2.set_title('Efficiency - Bar Chart', fontsize=12, fontweight='bold')
    ax2.set_xticks(x)
    ax2.set_xticklabels(results['labels'], rotation=15, ha='right')
    ax2.legend()
    ax2.grid(axis='y', alpha=0.3)

    # Plot 3: Path Cost - Line Chart
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(x, results['AStar_cost'], 'o-', label='A*',
             color='#2ecc71', linewidth=2, markersize=8)
    ax3.plot(x, results['BestF_cost'], 's-', label='Best-First',
             color='#e74c3c', linewidth=2, markersize=8)
    ax3.set_xlabel(x_label, fontsize=11)
    ax3.set_ylabel('Path Cost', fontsize=11)
    ax3.set_title('Solution Quality - Line Chart',
                  fontsize=12, fontweight='bold')
    ax3.set_xticks(x)
    ax3.set_xticklabels(results['labels'], rotation=15, ha='right')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Plot 4: Nodes Explored - Line Chart
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(x, results['AStar_nodes'], 'o-', label='A*',
             color='#2ecc71', linewidth=2, markersize=8)
    ax4.plot(x, results['BestF_nodes'], 's-', label='Best-First',
             color='#e74c3c', linewidth=2, markersize=8)
    ax4.set_xlabel(x_label, fontsize=11)
    ax4.set_ylabel('Nodes Explored', fontsize=11)
    ax4.set_title('Efficiency - Line Chart', fontsize=12, fontweight='bold')
    ax4.set_xticks(x)
    ax4.set_xticklabels(results['labels'], rotation=15, ha='right')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    # Plot 5: Efficiency Ratio (BestF/AStar nodes)
    ax5 = fig.add_subplot(gs[2, 0])
    ratios = [b/a if a > 0 else 1 for a, b in zip(
        results['AStar_nodes'], results['BestF_nodes'])]
    colors = ['#e74c3c' if r > 1 else '#2ecc71' for r in ratios]
    ax5.bar(x, ratios, width=0.6, color=colors, alpha=0.7)
    ax5.axhline(y=1, color='black', linestyle='--',
                linewidth=1, label='Equal Performance')
    ax5.set_xlabel(x_label, fontsize=11)
    ax5.set_ylabel('Ratio (BestF / A*)', fontsize=11)
    ax5.set_title('Efficiency Ratio: Best-First vs A*\n(>1 means A* is better)',
                  fontsize=12, fontweight='bold')
    ax5.set_xticks(x)
    ax5.set_xticklabels(results['labels'], rotation=15, ha='right')
    ax5.legend()
    ax5.grid(axis='y', alpha=0.3)

    # Plot 6: Solution Quality Difference
    ax6 = fig.add_subplot(gs[2, 1])
    cost_diff = [b - a for a, b in zip(
        results['AStar_cost'], results['BestF_cost'])]
    colors = ['#e74c3c' if d > 0 else '#2ecc71' for d in cost_diff]
    ax6.bar(x, cost_diff, width=0.6, color=colors, alpha=0.7)
    ax6.axhline(y=0, color='black', linestyle='--',
                linewidth=1, label='Equal Cost')
    ax6.set_xlabel(x_label, fontsize=11)
    ax6.set_ylabel('Cost Difference (BestF - A*)', fontsize=11)
    ax6.set_title('Solution Quality Difference\n(>0 means A* found better path)',
                  fontsize=12, fontweight='bold')
    ax6.set_xticks(x)
    ax6.set_xticklabels(results['labels'], rotation=15, ha='right')
    ax6.legend()
    ax6.grid(axis='y', alpha=0.3)

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
    labels = ['3', '4', '5', '6', '7-9']
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

    # Create summary comparison
    print("\n=== Creating Summary Comparisons ===")
    create_summary_plot(output_dir, all_results)

    print("\n✓ All evaluations complete! Check the 'results' directory for plots.")


def create_summary_plot(output_dir, all_results):
    """Create comprehensive summary plots showing overall performance across all tests"""

    # Calculate aggregate statistics
    categories = []
    avg_astar_cost = []
    avg_bestf_cost = []
    avg_astar_nodes = []
    avg_bestf_nodes = []
    total_astar_cost = []
    total_bestf_cost = []
    total_astar_nodes = []
    total_bestf_nodes = []

    for cat_name, results in all_results:
        categories.append(cat_name)
        avg_astar_cost.append(np.mean(results['AStar_cost']))
        avg_bestf_cost.append(np.mean(results['BestF_cost']))
        avg_astar_nodes.append(np.mean(results['AStar_nodes']))
        avg_bestf_nodes.append(np.mean(results['BestF_nodes']))
        total_astar_cost.append(np.sum(results['AStar_cost']))
        total_bestf_cost.append(np.sum(results['BestF_cost']))
        total_astar_nodes.append(np.sum(results['AStar_nodes']))
        total_bestf_nodes.append(np.sum(results['BestF_nodes']))

    # Create overall comparison figure
    fig = plt.figure(figsize=(18, 12))
    gs = fig.add_gridspec(3, 3, hspace=0.35, wspace=0.35)
    fig.suptitle('Overall Performance Comparison: A* vs Best-First Search (All 25 Tests)',
                 fontsize=20, fontweight='bold', y=0.98)

    x = np.arange(len(categories))
    width = 0.35

    # Plot 1: Average Path Cost by Category
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.bar(x - width/2, avg_astar_cost, width,
            label='A*', color='#2ecc71', alpha=0.8)
    ax1.bar(x + width/2, avg_bestf_cost, width,
            label='Best-First', color='#e74c3c', alpha=0.8)
    ax1.set_ylabel('Average Path Cost', fontsize=11)
    ax1.set_title('Average Solution Quality by Category',
                  fontsize=12, fontweight='bold')
    ax1.set_xticks(x)
    ax1.set_xticklabels(categories, rotation=20, ha='right', fontsize=9)
    ax1.legend()
    ax1.grid(axis='y', alpha=0.3)

    # Plot 2: Average Nodes Explored by Category
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.bar(x - width/2, avg_astar_nodes, width,
            label='A*', color='#2ecc71', alpha=0.8)
    ax2.bar(x + width/2, avg_bestf_nodes, width,
            label='Best-First', color='#e74c3c', alpha=0.8)
    ax2.set_ylabel('Average Nodes Explored', fontsize=11)
    ax2.set_title('Average Efficiency by Category',
                  fontsize=12, fontweight='bold')
    ax2.set_xticks(x)
    ax2.set_xticklabels(categories, rotation=20, ha='right', fontsize=9)
    ax2.legend()
    ax2.grid(axis='y', alpha=0.3)

    # Plot 3: Pie Chart - Total Nodes Explored
    ax3 = fig.add_subplot(gs[0, 2])
    total_all_astar = sum(total_astar_nodes)
    total_all_bestf = sum(total_bestf_nodes)
    ax3.pie([total_all_astar, total_all_bestf], labels=['A*', 'Best-First'],
            colors=['#2ecc71', '#e74c3c'], autopct='%1.1f%%', startangle=90,
            textprops={'fontsize': 12, 'fontweight': 'bold'})
    ax3.set_title(f'Total Nodes Explored\nAcross All Tests\n(Total: {total_all_astar + total_all_bestf:,})',
                  fontsize=12, fontweight='bold')

    # Plot 4: Efficiency Ratio by Category
    ax4 = fig.add_subplot(gs[1, 0])
    efficiency_ratios = [b/a if a > 0 else 1 for a,
                         b in zip(avg_astar_nodes, avg_bestf_nodes)]
    colors = ['#e74c3c' if r > 1 else '#2ecc71' for r in efficiency_ratios]
    bars = ax4.bar(x, efficiency_ratios, width=0.6, color=colors, alpha=0.7)
    ax4.axhline(y=1, color='black', linestyle='--',
                linewidth=2, label='Equal Performance')
    ax4.set_ylabel('Ratio (BestF / A*)', fontsize=11)
    ax4.set_title('Efficiency Ratio by Category\n(>1 means A* explores fewer nodes)',
                  fontsize=12, fontweight='bold')
    ax4.set_xticks(x)
    ax4.set_xticklabels(categories, rotation=20, ha='right', fontsize=9)
    # Add value labels on bars
    for i, (bar, val) in enumerate(zip(bars, efficiency_ratios)):
        ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.05,
                 f'{val:.2f}x', ha='center', va='bottom', fontsize=9, fontweight='bold')
    ax4.legend()
    ax4.grid(axis='y', alpha=0.3)

    # Plot 5: Solution Quality Difference
    ax5 = fig.add_subplot(gs[1, 1])
    cost_diffs = [b - a for a, b in zip(avg_astar_cost, avg_bestf_cost)]
    colors = ['#e74c3c' if d > 0 else '#2ecc71' if d <
              0 else '#95a5a6' for d in cost_diffs]
    bars = ax5.bar(x, cost_diffs, width=0.6, color=colors, alpha=0.7)
    ax5.axhline(y=0, color='black', linestyle='--',
                linewidth=2, label='Equal Cost')
    ax5.set_ylabel('Cost Difference (BestF - A*)', fontsize=11)
    ax5.set_title('Average Solution Quality Difference\n(>0 means A* found better paths)',
                  fontsize=12, fontweight='bold')
    ax5.set_xticks(x)
    ax5.set_xticklabels(categories, rotation=20, ha='right', fontsize=9)
    # Add value labels on bars
    for bar, val in zip(bars, cost_diffs):
        if val != 0:
            ax5.text(bar.get_x() + bar.get_width()/2,
                     bar.get_height() + (0.1 if val > 0 else -0.1),
                     f'{val:.2f}', ha='center',
                     va='bottom' if val > 0 else 'top',
                     fontsize=9, fontweight='bold')
    ax5.legend()
    ax5.grid(axis='y', alpha=0.3)

    # Plot 6: Scatter Plot - Cost vs Nodes
    ax6 = fig.add_subplot(gs[1, 2])
    all_astar_costs = []
    all_astar_nodes_list = []
    all_bestf_costs = []
    all_bestf_nodes_list = []
    for _, results in all_results:
        all_astar_costs.extend(results['AStar_cost'])
        all_astar_nodes_list.extend(results['AStar_nodes'])
        all_bestf_costs.extend(results['BestF_cost'])
        all_bestf_nodes_list.extend(results['BestF_nodes'])

    ax6.scatter(all_astar_nodes_list, all_astar_costs,
                alpha=0.6, s=80, c='#2ecc71', label='A*', edgecolors='black', linewidth=0.5)
    ax6.scatter(all_bestf_nodes_list, all_bestf_costs,
                alpha=0.6, s=80, c='#e74c3c', label='Best-First',
                marker='s', edgecolors='black', linewidth=0.5)
    ax6.set_xlabel('Nodes Explored', fontsize=11)
    ax6.set_ylabel('Path Cost', fontsize=11)
    ax6.set_title('Cost vs Efficiency Trade-off\n(All 25 Tests)',
                  fontsize=12, fontweight='bold')
    ax6.legend()
    ax6.grid(True, alpha=0.3)

    # Plot 7: Heatmap of Performance
    ax7 = fig.add_subplot(gs[2, 0])
    # Normalize data for heatmap
    heatmap_data = np.array([
        [np.mean(r['AStar_cost']) for _, r in all_results],
        [np.mean(r['BestF_cost']) for _, r in all_results]
    ])
    im = ax7.imshow(heatmap_data, cmap='RdYlGn_r', aspect='auto')
    ax7.set_yticks([0, 1])
    ax7.set_yticklabels(['A*', 'Best-First'])
    ax7.set_xticks(x)
    ax7.set_xticklabels(categories, rotation=20, ha='right', fontsize=9)
    ax7.set_title('Average Path Cost Heatmap', fontsize=12, fontweight='bold')
    plt.colorbar(im, ax=ax7, label='Path Cost')

    # Plot 8: Cumulative Tests - A* Wins
    ax8 = fig.add_subplot(gs[2, 1])
    astar_wins = 0
    bestf_wins = 0
    ties = 0
    for _, results in all_results:
        for a_cost, b_cost in zip(results['AStar_cost'], results['BestF_cost']):
            if abs(a_cost - b_cost) < 0.01:
                ties += 1
            elif a_cost < b_cost:
                astar_wins += 1
            else:
                bestf_wins += 1

    wedges, texts, autotexts = ax8.pie([astar_wins, bestf_wins, ties],
                                       labels=['A* Better',
                                               'BestF Better', 'Tied'],
                                       colors=['#2ecc71',
                                               '#e74c3c', '#95a5a6'],
                                       autopct='%1.1f%%', startangle=90,
                                       textprops={'fontsize': 11})
    for autotext in autotexts:
        autotext.set_color('white')
        autotext.set_fontweight('bold')
    ax8.set_title(f'Solution Quality Wins\n(Total: {astar_wins + bestf_wins + ties} tests)',
                  fontsize=12, fontweight='bold')

    # Plot 9: Statistics Summary Table
    ax9 = fig.add_subplot(gs[2, 2])
    ax9.axis('off')

    stats_text = f"""
    OVERALL STATISTICS
    ═══════════════════════════════

    Total Tests: 25

    PATH COST:
    • A* Average: {np.mean(all_astar_costs):.2f}
    • BestF Average: {np.mean(all_bestf_costs):.2f}
    • A* Best: {min(all_astar_costs):.2f}
    • BestF Best: {min(all_bestf_costs):.2f}

    NODES EXPLORED:
    • A* Average: {np.mean(all_astar_nodes_list):.0f}
    • BestF Average: {np.mean(all_bestf_nodes_list):.0f}
    • A* Total: {sum(all_astar_nodes_list):,}
    • BestF Total: {sum(all_bestf_nodes_list):,}

    WINS:
    • A* Better Quality: {astar_wins} tests
    • BestF Better Quality: {bestf_wins} tests
    • Tied: {ties} tests
    """

    ax9.text(0.1, 0.95, stats_text, transform=ax9.transAxes,
             fontsize=10, verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))

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
    fig.suptitle('Detailed View: All 25 Tests Across 5 Categories',
                 fontsize=20, fontweight='bold', y=0.98)

    # Flatten all results
    all_labels = []
    all_astar_cost = []
    all_bestf_cost = []
    all_astar_nodes = []
    all_bestf_nodes = []
    category_boundaries = [0]

    for cat_name, results in all_results:
        for i, label in enumerate(results['labels']):
            all_labels.append(f"{cat_name[:4]}-{label}")
            all_astar_cost.append(results['AStar_cost'][i])
            all_bestf_cost.append(results['BestF_cost'][i])
            all_astar_nodes.append(results['AStar_nodes'][i])
            all_bestf_nodes.append(results['BestF_nodes'][i])
        category_boundaries.append(len(all_labels))

    x = np.arange(len(all_labels))

    # Plot 1: All Path Costs
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(x, all_astar_cost, 'o-', label='A*',
             color='#2ecc71', linewidth=2, markersize=6, alpha=0.8)
    ax1.plot(x, all_bestf_cost, 's-', label='Best-First',
             color='#e74c3c', linewidth=2, markersize=6, alpha=0.8)

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

    ax1.set_xlabel('Test Number', fontsize=13)
    ax1.set_ylabel('Path Cost', fontsize=13)
    ax1.set_title('Solution Quality Across All 25 Tests',
                  fontsize=14, fontweight='bold')
    ax1.set_xticks(x)
    ax1.set_xticklabels(range(1, 26), fontsize=8)
    ax1.legend(fontsize=12)
    ax1.grid(True, alpha=0.3)

    # Plot 2: All Nodes Explored
    ax2 = fig.add_subplot(gs[1, 0])
    width = 0.4
    ax2.bar(x - width/2, all_astar_nodes, width, label='A*',
            color='#2ecc71', alpha=0.7)
    ax2.bar(x + width/2, all_bestf_nodes, width, label='Best-First',
            color='#e74c3c', alpha=0.7)

    # Add vertical lines to separate categories
    for boundary in category_boundaries[1:-1]:
        ax2.axvline(x=boundary-0.5, color='gray',
                    linestyle='--', linewidth=1, alpha=0.5)

    ax2.set_xlabel('Test Number', fontsize=12)
    ax2.set_ylabel('Nodes Explored', fontsize=12)
    ax2.set_title('Efficiency Across All 25 Tests',
                  fontsize=13, fontweight='bold')
    ax2.set_xticks(x)
    ax2.set_xticklabels(range(1, 26), fontsize=8)
    ax2.legend(fontsize=11)
    ax2.grid(axis='y', alpha=0.3)

    # Plot 3: Efficiency Improvement (percentage)
    ax3 = fig.add_subplot(gs[1, 1])
    improvements = [(b - a) / b * 100 if b > 0 else 0
                    for a, b in zip(all_astar_nodes, all_bestf_nodes)]
    colors = ['#2ecc71' if imp > 0 else '#e74c3c' if imp < 0 else '#95a5a6'
              for imp in improvements]

    ax3.bar(x, improvements, width=0.6, color=colors, alpha=0.7)
    ax3.axhline(y=0, color='black', linestyle='-', linewidth=2)

    # Add vertical lines to separate categories
    for boundary in category_boundaries[1:-1]:
        ax3.axvline(x=boundary-0.5, color='gray',
                    linestyle='--', linewidth=1, alpha=0.5)

    ax3.set_xlabel('Test Number', fontsize=12)
    ax3.set_ylabel('A* Improvement (%)', fontsize=12)
    ax3.set_title('A* Efficiency Improvement Over Best-First\n(% fewer nodes explored)',
                  fontsize=13, fontweight='bold')
    ax3.set_xticks(x)
    ax3.set_xticklabels(range(1, 26), fontsize=8)
    ax3.grid(axis='y', alpha=0.3)

    plt.savefig(f"{output_dir}/00_all_tests_detailed.png",
                dpi=300, bbox_inches='tight')
    print(f"Saved plot: {output_dir}/00_all_tests_detailed.png")
    plt.close()


if __name__ == "__main__":
    main()
