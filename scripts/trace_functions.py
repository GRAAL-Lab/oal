import re
import pandas as pd
import matplotlib.pyplot as plt
from collections import defaultdict


def get_traces(parsed_data, index=None, name=None):
    # If both index and name are provided, raise an error as both are mutually exclusive
    if index is not None and name is not None:
        raise ValueError("You can only provide one of index or name, not both.")
    
    # If index is provided, return the trace at that index
    if index is not None:
        if index < 0 or index >= len(parsed_data):
            raise IndexError("Index out of range.")
        return parsed_data[index]
    
    # If name is provided, return all traces with that name
    if name is not None:
        result = [trace for trace in parsed_data if trace['trace_name'] == name]
        return result
    
    # If neither index nor name is provided, return all traces
    return parsed_data

def plot_trace(trace, ax, color='blue'):
    """
    Plots the given trace's nodes on an existing plot (ax).
    If no axis is provided, it creates a new plot.
    
    :param trace: The trace data (dictionary containing 'trace_name' and 'nodes').
    :param ax: Existing Matplotlib axis object to plot on. If None, a new plot is created.
    """
    # Extract positions (x, y) of the nodes
    x = [node['position'][0] for node in trace['nodes']]
    y = [node['position'][1] for node in trace['nodes']]
    
    
    # Plot the nodes
    ax.plot(x, y, marker='o', color=color, linestyle='-', markersize=6)
    

def parse_traces(file_path):
    traces = []
    
    with open(file_path, 'r') as f:
        content = f.read().strip()
    
    # Split the content by the delimiter '---' and remove any empty blocks
    blocks = [block.strip() for block in content.split('---') if block.strip()]
    
    for block in blocks:
        lines = block.splitlines()
        if not lines:
            continue
        
        # Get the trace description by removing "Trace_" prefix if present
        header_line = lines[0].strip()
        description = header_line[len("Trace_"):] if header_line.startswith("Trace_") else header_line
        
        nodes = []
        # Process each subsequent line as a node entry
        for node_line in lines[1:]:
            node_line = node_line.strip()
            if not node_line:
                continue
            
            # Split the line by '_' and filter out any empty tokens
            tokens = [t for t in node_line.split('_') if t]
            
            try:
                # Create a node with required attributes and default None for obstacle and vx
                node = {
                    'index': int(tokens[1]),
                    'position': (float(tokens[3]), float(tokens[4])),
                    'heading': float(tokens[6]),
                    'time': float(tokens[8]),
                    'speed': float(tokens[10]),
                    'obstacle': None,
                    'vx': None
                }
                # Check if obstacle and vx tokens are present.
                if len(tokens) >= 15 and tokens[11] == "Obstacle":
                    node['obstacle'] = tokens[12]
                    # Expect tokens[13] to be "Vx" and tokens[14] the corresponding value.
                    if len(tokens) >= 15 and tokens[13] == "Vx":
                        node['vx'] = tokens[14]
            except Exception:
                # Skip any node entries that raise errors during parsing
                continue
            
            nodes.append(node)
        
        traces.append({'description': description, 'nodes': nodes})
    
    return traces

# def parse_trace_data(file_path):
#     traces = []
    
#     with open(file_path, 'r') as file:
#         content = file.read()
        
#         # Split the content by '---' to isolate each trace
#         trace_blocks = content.strip().split('---')[1:]
        
#         for block in trace_blocks:
#             lines = block.strip().split('\n')
#             trace_name = lines[0].strip()
#             nodes = []
            
#             for line in lines[1:]:
#                 # This regex makes both the obstacle and Vx parts fully optional
#                 match = re.match(
#                     r'_Node_(\d+)_Position_([\d.-]+)_([\d.-]+)_Heading_([\d.-]+)_Time_([\d.-]+)_Speed_([\d.-]+)'
#                     r'(?:_Obstacle_([\w\d]+))?'
#                     r'(?:_Vx_([\w\d]+))?', 
#                     line
#                 )
                
#                 if match:
#                     obstacle = match.group(7) if match.group(7) is not None else 'none'
#                     vx = match.group(8) if match.group(8) is not None else 'none'
                    
#                     node = {
#                         'node': int(match.group(1)),
#                         'position': (float(match.group(2)), float(match.group(3))),
#                         'heading': float(match.group(4)),
#                         'time': float(match.group(5)),
#                         'speed': float(match.group(6)),
#                         'obstacle': obstacle,
#                         'vx': vx
#                     }
#                     nodes.append(node)
            
#             traces.append({'trace_name': trace_name, 'nodes': nodes})
    
#     return traces


