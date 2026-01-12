#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv('multi_start/multi_start_results.csv')
successful = df[df['success'] == 1]

print(f"Loaded {len(df)} attempts, {len(successful)} successful")

# Create figure with subplots
fig, axes = plt.subplots(2, 2, figsize=(12, 10))
fig.suptitle('Multi-Start RRT Performance Analysis', fontsize=16, fontweight='bold')

# 1. Success rate by attempt
success_by_attempt = df.groupby('attempt')['success'].mean() * 100
colors = ['green' if x > 0 else 'red' for x in success_by_attempt.values]
axes[0, 0].bar(success_by_attempt.index, success_by_attempt.values, color=colors, alpha=0.7, edgecolor='black')
axes[0, 0].set_xlabel('Attempt Number', fontsize=11)
axes[0, 0].set_ylabel('Success Rate (%)', fontsize=11)
axes[0, 0].set_title('Success Rate by Attempt', fontsize=12, fontweight='bold')
axes[0, 0].set_ylim([0, 105])
axes[0, 0].set_xticks(range(int(df['attempt'].max()) + 1))
axes[0, 0].grid(axis='y', alpha=0.3)

# Add text for overall success rate
overall_success = (df['success'].sum() / len(df)) * 100
axes[0, 0].text(0.5, 0.95, f'Overall: {overall_success:.1f}%', 
                transform=axes[0, 0].transAxes, ha='center', va='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

# 2. Planning time distribution
if len(successful) > 0:
    axes[0, 1].hist(successful['planning_time_sec'], bins=min(10, len(successful)), 
                    color='blue', alpha=0.7, edgecolor='black')
    axes[0, 1].set_xlabel('Planning Time (s)', fontsize=11)
    axes[0, 1].set_ylabel('Count', fontsize=11)
    axes[0, 1].set_title('Planning Time Distribution', fontsize=12, fontweight='bold')
    mean_time = successful['planning_time_sec'].mean()
    axes[0, 1].axvline(mean_time, color='red', linestyle='--', linewidth=2, label=f'Mean: {mean_time:.3f}s')
    axes[0, 1].legend()
    axes[0, 1].grid(axis='y', alpha=0.3)

# 3. Path length distribution
if len(successful) > 0:
    axes[1, 0].hist(successful['path_length'], bins=min(10, len(successful)), 
                    color='orange', alpha=0.7, edgecolor='black')
    axes[1, 0].set_xlabel('Path Length (radians)', fontsize=11)
    axes[1, 0].set_ylabel('Count', fontsize=11)
    title_text = 'Path Length Distribution\n'
    formula_text = r'$\ell = \sum_{i=1}^{n} \sqrt{\sum_{j=1}^{7}(q_j^{(i)} - q_j^{(i-1)})^2}$'
    axes[1, 0].set_title(title_text + formula_text, fontsize=11, fontweight='bold')
    mean_length = successful['path_length'].mean()
    best_length = successful['path_length'].min()
    axes[1, 0].axvline(mean_length, color='red', linestyle='--', linewidth=2, label=f'Mean: {mean_length:.2f}')
    axes[1, 0].axvline(best_length, color='green', linestyle='--', linewidth=2, label=f'Best: {best_length:.2f}')
    axes[1, 0].legend()
    axes[1, 0].grid(axis='y', alpha=0.3)

# 4. Cost vs Planning Time scatter
if len(successful) > 0:
    scatter = axes[1, 1].scatter(successful['planning_time_sec'], successful['total_cost'], 
                                 c=successful['path_length'], cmap='viridis',
                                 alpha=0.7, s=150, edgecolors='black', linewidth=1)
    axes[1, 1].set_xlabel('Planning Time (s)', fontsize=11)
    axes[1, 1].set_ylabel('Total Cost', fontsize=11)
    axes[1, 1].set_title('Cost vs Planning Time', fontsize=12, fontweight='bold')
    axes[1, 1].grid(alpha=0.3)
    
    # Add colorbar
    cbar = plt.colorbar(scatter, ax=axes[1, 1])
    cbar.set_label('Path Length', fontsize=10)
    
    # Annotate best point
    best_idx = successful['total_cost'].idxmin()
    best_time = successful.loc[best_idx, 'planning_time_sec']
    best_cost = successful.loc[best_idx, 'total_cost']
    axes[1, 1].scatter([best_time], [best_cost], color='red', s=300, 
                       marker='*', edgecolors='black', linewidth=2, label='Best', zorder=5)
    axes[1, 1].legend()

plt.tight_layout()
plt.savefig('multi_start/multi_start_analysis.png', dpi=300, bbox_inches='tight')
print("✓ Saved plot to: multi_start/multi_start_analysis.png")
plt.show()
