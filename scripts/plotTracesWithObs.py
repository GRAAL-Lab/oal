import sys
import argparse
import matplotlib
matplotlib.use('TkAgg')  # Ensure GUI backend

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider

from plot_functions import *  # e.g., plot_init
from obs_functions import *   # e.g., get_obstacle_row_by_name, plot_obstacle, compute_obstacle_position_at_time, parse_obstacle_file
from trace_functions import * # e.g., parse_traces

# --- Command-line argument parsing ---
parser = argparse.ArgumentParser(description="Interactive trace and obstacle plotting.")
parser.add_argument("nodesFile", choices=["completeNodes", "failedNodes", "validNodes", "notValidNodes"],
                    help="Choose which nodes file to use.")
args = parser.parse_args()

# --- File paths ---
obstFile = '/home/graal/ros2_ws/log/avoidance_logs/obstacles/obstacles.txt'

# Choose the nodes file based on the command-line argument.
if args.nodesFile == "completeNodes":
    nodesFile = '/home/graal/ros2_ws/log/avoidance_logs/nodes/completePathNodes.txt'
elif args.nodesFile == "failedNodes":
    nodesFile = '/home/graal/ros2_ws/log/avoidance_logs/nodes/failedPathNodes.txt'
elif args.nodesFile == "validNodes":
    nodesFile = '/home/graal/ros2_ws/log/avoidance_logs/nodes/validPathNodes.txt'
elif args.nodesFile == "notValidNodes":
    nodesFile = '/home/graal/ros2_ws/log/avoidance_logs/nodes/notValidPathNodes.txt'

# --- Parse data ---
traces = parse_traces(nodesFile)
obstacles_df = parse_obstacle_file(obstFile)

# --- Initialize figure and axis ---
fig, ax = plot_init()

# --- Containers for curves and obstacles ---
curves = []                
# For each trace, we store a list of tuples:
# (orig_obstacle_row, node_time, polygon, text_obj)
trace_obstacles = []

# --- Process each trace ---
for trace in traces:
    # Extract trace positions
    x = [node['position'][0] for node in trace['nodes']]
    y = [node['position'][1] for node in trace['nodes']]
    
    # Plot the trace line (initially hidden)
    line, = ax.plot(x, y, marker='o', color='red', linestyle='-', markersize=6, visible=False)
    curves.append(line)
    
    # Process obstacles for this trace:
    obstacles_this_trace = []
    for node in trace['nodes']:
        if node['obstacle'] != 'none':
            obs_row = get_obstacle_row_by_name(obstacles_df, node['obstacle'])
            if obs_row is None:
                continue
            # Save a copy of the original obstacle row so that updates always start from the same base data.
            orig_obs_row = obs_row.copy()
            node_time = node['time']
            # Create the obstacle polygon and text based on the node's time.
            polygon, text_obj = plot_obstacle(orig_obs_row, t=node_time, color='blue')
            polygon.set_visible(False)
            text_obj.set_visible(False)
            ax.add_patch(polygon)
            ax.add_artist(text_obj)
            obstacles_this_trace.append((orig_obs_row, node_time, polygon, text_obj))
    trace_obstacles.append(obstacles_this_trace)

# --- Initially show the first trace and update its obstacles ---
if curves:
    curves[0].set_visible(True)
    for orig_obs_row, node_time, polygon, text_obj in trace_obstacles[0]:
        updated_row = compute_obstacle_position_at_time(orig_obs_row, node_time)
        new_coords = [
            (updated_row['Vx0_x'], updated_row['Vx0_y']),
            (updated_row['Vx1_x'], updated_row['Vx1_y']),
            (updated_row['Vx3_x'], updated_row['Vx3_y']),
            (updated_row['Vx2_x'], updated_row['Vx2_y'])
        ]
        polygon.set_xy(new_coords)
        text_obj.set_position((updated_row['pos_x'], updated_row['pos_y']))
        text_obj.set_text(f"{updated_row['obstacle_id']}\nt={node_time:.2f}")
        polygon.set_visible(True)
        text_obj.set_visible(True)

# --- Create slider for selecting trace ---
ax_slider = plt.axes([0.3, 0.05, 0.6, 0.03])
slider = Slider(ax_slider, 'Trace', 0, len(curves) - 1, valinit=0, valstep=1)

# --- Callback for slider ---
def update_plot(val):
    trace_idx = int(slider.val)
    
    # Toggle trace visibility: only show the selected trace.
    for i, curve in enumerate(curves):
        curve.set_visible(i == trace_idx)
    
    # Update obstacles: for the selected trace, update each obstacle's position based on its node time.
    for i, obs_list in enumerate(trace_obstacles):
        if i == trace_idx:
            for orig_obs_row, node_time, polygon, text_obj in obs_list:
                updated_row = compute_obstacle_position_at_time(orig_obs_row, node_time)
                new_coords = [
                    (updated_row['Vx0_x'], updated_row['Vx0_y']),
                    (updated_row['Vx1_x'], updated_row['Vx1_y']),
                    (updated_row['Vx3_x'], updated_row['Vx3_y']),
                    (updated_row['Vx2_x'], updated_row['Vx2_y'])
                ]
                polygon.set_xy(new_coords)
                text_obj.set_position((updated_row['pos_x'], updated_row['pos_y']))
                text_obj.set_text(f"{updated_row['obstacle_id']}\nt={node_time:.2f}")
                polygon.set_visible(True)
                text_obj.set_visible(True)
        else:
            for _, _, polygon, text_obj in obs_list:
                polygon.set_visible(False)
                text_obj.set_visible(False)
    
    plt.draw()

slider.on_changed(update_plot)

plt.show()
