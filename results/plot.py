#!/usr/bin/env python3
"""
Compare RRT performance before and after orientation constraint
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Data before constraint (from /tmp)
before_data = {
    'trial': [0, 0, 0, 0, 0, 0],
    'attempt': [0, 1, 2, 3, 4, 5],
    'success': [1, 1, 0, 1, 1, 1],
    'planning_time_sec': [0.4910, 0.4450, 1.1478, 0.8908, 0.4076, 1.4425],
    'path_length': [4.293, 3.126, 0.000, 4.907, 5.213, 3.761],
    'smoothness': [0.001, 0.001, 0.000, 0.000, 0.001, 0.000],
    'total_cost': [4.294, 3.127, 999.000, 4.907, 5.213, 3.762]
}

# Data after constraint
after_data = {
    'trial': [0, 0, 0, 0, 0, 0],
    'attempt': [0, 1, 2, 3, 4, 5],
    'success': [1, 1, 1, 1, 1, 1],
    'planning_time_sec': [0.0576, 0.0836, 0.0513, 0.0683, 0.0655, 0.0483],
    'path_length': [2.054, 2.052, 2.055, 2.054, 2.053, 2.053],
    'smoothness': [0.000, 0.000, 0.000, 0.000, 0.000, 0.000],
    'total_cost': [2.054, 2.053, 2.055, 2.054, 2.053, 2.053]
}

df_before = pd.DataFrame(before_data)
df_after = pd.DataFrame(after_data)

# Filter successful attempts
before_success = df_before[df_before['success'] == 1]
after_success = df_after[df_after['success'] == 1]

# Create figure
fig, axes = plt.subplots(2, 2, figsize=(12, 10))
fig.suptitle('RRT Performance: Before vs After Orientation Constraint (±23°)', 
             fontsize=14, fontweight='bold')

# 1. Success Rate Comparison
ax1 = axes[0, 0]
labels = ['Before\n(No Constraint)', 'After\n(Top-Down ±23°)']
success_rates = [
    df_before['success'].mean() * 100,
    df_after['success'].mean() * 100
]
colors = ['#ff6b6b', '#51cf66']
bars = ax1.bar(labels, success_rates, color=colors, edgecolor='black', linewidth=2)
ax1.set_ylabel('Success Rate (%)', fontsize=11)
ax1.set_title('Success Rate', fontsize=12, fontweight='bold')
ax1.set_ylim([0, 110])
for bar, rate in zip(bars, success_rates):
    ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 2, 
             f'{rate:.0f}%', ha='center', va='bottom', fontsize=12, fontweight='bold')

# 2. Planning Time Comparison
ax2 = axes[0, 1]
positions = [0, 1]
bp1 = ax2.boxplot([before_success['planning_time_sec']], positions=[0], widths=0.6,
                   patch_artist=True, boxprops=dict(facecolor='#ff6b6b', alpha=0.7))
bp2 = ax2.boxplot([after_success['planning_time_sec']], positions=[1], widths=0.6,
                   patch_artist=True, boxprops=dict(facecolor='#51cf66', alpha=0.7))
ax2.set_xticks([0, 1])
ax2.set_xticklabels(['Before', 'After'])
ax2.set_ylabel('Planning Time (s)', fontsize=11)
ax2.set_title('Planning Time Distribution', fontsize=12, fontweight='bold')
ax2.grid(axis='y', alpha=0.3)

# Add mean annotations
mean_before = before_success['planning_time_sec'].mean()
mean_after = after_success['planning_time_sec'].mean()
ax2.text(0, mean_before + 0.1, f'μ={mean_before:.3f}s', ha='center', fontsize=10)
ax2.text(1, mean_after + 0.01, f'μ={mean_after:.3f}s', ha='center', fontsize=10)

# 3. Path Length Comparison
ax3 = axes[1, 0]
bp3 = ax3.boxplot([before_success['path_length']], positions=[0], widths=0.6,
                   patch_artist=True, boxprops=dict(facecolor='#ff6b6b', alpha=0.7))
bp4 = ax3.boxplot([after_success['path_length']], positions=[1], widths=0.6,
                   patch_artist=True, boxprops=dict(facecolor='#51cf66', alpha=0.7))
ax3.set_xticks([0, 1])
ax3.set_xticklabels(['Before', 'After'])
ax3.set_ylabel('Path Length (rad)', fontsize=11)
ax3.set_title('Path Length Distribution', fontsize=12, fontweight='bold')
ax3.grid(axis='y', alpha=0.3)

mean_len_before = before_success['path_length'].mean()
mean_len_after = after_success['path_length'].mean()
ax3.text(0, mean_len_before + 0.2, f'μ={mean_len_before:.2f}', ha='center', fontsize=10)
ax3.text(1, mean_len_after + 0.1, f'μ={mean_len_after:.2f}', ha='center', fontsize=10)

# 4. Summary Statistics Table
ax4 = axes[1, 1]
ax4.axis('off')

summary_data = [
    ['Metric', 'Before', 'After', 'Change'],
    ['Success Rate', f'{df_before["success"].mean()*100:.0f}%', 
     f'{df_after["success"].mean()*100:.0f}%', 
     f'+{(df_after["success"].mean() - df_before["success"].mean())*100:.0f}%'],
    ['Avg Planning Time', f'{mean_before:.3f}s', f'{mean_after:.3f}s', 
     f'{((mean_after-mean_before)/mean_before)*100:.0f}%'],
    ['Avg Path Length', f'{mean_len_before:.2f}', f'{mean_len_after:.2f}',
     f'{((mean_len_after-mean_len_before)/mean_len_before)*100:.0f}%'],
    ['Best Cost', f'{before_success["total_cost"].min():.2f}', 
     f'{after_success["total_cost"].min():.2f}', ''],
    ['Constraint', 'None', '±23° top-down', ''],
]

table = ax4.table(cellText=summary_data, loc='center', cellLoc='center',
                  colWidths=[0.3, 0.2, 0.2, 0.2])
table.auto_set_font_size(False)
table.set_fontsize(11)
table.scale(1.2, 2)

# Style header row
for j in range(4):
    table[(0, j)].set_facecolor('#4a4a4a')
    table[(0, j)].set_text_props(color='white', fontweight='bold')

ax4.set_title('Summary Statistics', fontsize=12, fontweight='bold', pad=20)

plt.tight_layout()
plt.savefig('orientation_constraint_comparison.png', dpi=300, bbox_inches='tight')
print("✓ Saved: orientation_constraint_comparison.png")
plt.show()
