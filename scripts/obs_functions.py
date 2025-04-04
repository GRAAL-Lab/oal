import re
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

def print_obstacle_names(df: pd.DataFrame) -> None:
    """
    Prints the names (IDs) of all obstacles in the DataFrame.
    
    Args:
        df (pd.DataFrame): The DataFrame containing obstacle data.
    """
    obstacle_names = df['obstacle_id'].unique()
    print("Obstacle Names (IDs):")
    for name in obstacle_names:
        print(name)

def get_obstacle_row_by_name(df: pd.DataFrame, obstacle_name: str):
    """
    Retrieve the row of an obstacle from the DataFrame by its name (ID).
    
    Args:
        df (pd.DataFrame): The DataFrame containing obstacle data.
        obstacle_name (str): The name (ID) of the obstacle to retrieve.
    
    Returns:
        pd.Series: The row of the obstacle from the DataFrame, or None if not found.
    """
    obstacle_row = df[df['obstacle_id'] == obstacle_name]
    if obstacle_row.empty:
        #print(f"Obstacle with ID '{obstacle_name}' not found.")
        return None
    else:
        return obstacle_row.iloc[0]  # Returns the first (and only) row for that obstacle


def plot_obstacle(obstacle_row, t=0, color='blue'):
    """
    Plots an obstacle as a polygon defined by its four Vx points at time 't' on an existing matplotlib plot.
    Returns the Polygon and its associated text object.
    
    Args:
        obstacle_row (pd.Series): A row from the DataFrame representing a single obstacle.
        t (float): The time at which to compute the new position of the obstacle.
        color (str): The color of the obstacle's polygon.
        
    Returns:
        tuple: (polygon, text_obj)
    """
    # Compute obstacle position at time t using the time-dependent position
    obstacle_row = compute_obstacle_position_at_time(obstacle_row, t)

    # Define the polygon vertices from the Vx points
    x_points = [
        obstacle_row['Vx0_x'],
        obstacle_row['Vx1_x'],
        obstacle_row['Vx3_x'],
        obstacle_row['Vx2_x'],
        obstacle_row['Vx0_x']  # Close the loop
    ]
    y_points = [
        obstacle_row['Vx0_y'],
        obstacle_row['Vx1_y'],
        obstacle_row['Vx3_y'],
        obstacle_row['Vx2_y'],
        obstacle_row['Vx0_y']  # Close the loop
    ]
    
    # Create the polygon object (do not add it to the axis here)
    polygon = Polygon(list(zip(x_points, y_points)), facecolor=color, alpha=0.3, edgecolor='black')
    
    # Create a text label for the obstacle (this returns a Text object)
    text_obj = plt.text(obstacle_row['pos_x'], obstacle_row['pos_y'],
                        f'{obstacle_row["obstacle_id"]}\nt={t}',
                        ha='center', va='center', color='black', fontsize=10)
    
    return polygon, text_obj



def compute_obstacle_position_at_time(obstacle_row: pd.Series, t: float):
    """
    Compute the new position of an obstacle and its bounding box vertices at a given time 't',
    assuming constant velocity.

    Args:
        obstacle_row (pd.Series): A row from the DataFrame representing a single obstacle.
        t (float): The time at which to compute the new position.

    Returns:
        pd.Series: Updated obstacle data row with new position and bounding box at time 't'.
    """

    obs = obstacle_row.copy()
    
    # Extract initial positions and velocities
    x0, y0 = obs['pos_x'], obs['pos_y']
    vx, vy = float(obs['vel_x']), float(obs['vel_y'])  # Assuming velocities are in separate columns for vx, vy
    
    # Calculate new position for the center of the obstacle
    new_x = x0 + vx * t
    new_y = y0 + vy * t
    
    # Extract and update bounding box vertices (Vx0, Vx1, Vx2, Vx3) with the new positions
    for vertex in ['Vx0', 'Vx1', 'Vx2', 'Vx3']:
        v_x = obs[f'{vertex}_x']
        v_y = obs[f'{vertex}_y']
        
        # Update each vertex using velocity and time
        updated_vx = v_x + vx * t
        updated_vy = v_y + vy * t
        
        # Update the original row fields for each vertex
        obs[f'{vertex}_x'] = updated_vx
        obs[f'{vertex}_y'] = updated_vy
    
    # Update obstacle row with the new center position
    obs['pos_x'] = new_x
    obs['pos_y'] = new_y
    obs['computed_time'] = t  # Add computed time for reference
    
    return obs



def parse_obstacle_file(file_path: str) -> pd.DataFrame:
    """
    Parses a file containing obstacle data in a specific encoded format.
    
    Args:
        file_path (str): Path to the file to be parsed.
    
    Returns:
        pd.DataFrame: DataFrame containing structured obstacle data.
    """
    # Regular expression pattern to parse each line
    pattern = re.compile(
        r'Obstacle_(?P<obstacle_id>unknown\d+)_Position_'
        r'(?P<pos_x>[-\d.]+)_(?P<pos_y>[-\d.]+)_'
        r'Heading_(?P<heading>[-\d.]+)_Velocity_'
        r'(?P<vel_x>[-\d.]+)_(?P<vel_y>[-\d.]+)_'
        r'Vx0_(?P<Vx0_x>[-\d.]+)_(?P<Vx0_y>[-\d.]+)_'
        r'Vx1_(?P<Vx1_x>[-\d.]+)_(?P<Vx1_y>[-\d.]+)_'
        r'Vx2_(?P<Vx2_x>[-\d.]+)_(?P<Vx2_y>[-\d.]+)_'
        r'Vx3_(?P<Vx3_x>[-\d.]+)_(?P<Vx3_y>[-\d.]+)'
    )

    # List to hold parsed data
    data = []

    # Read and parse the file
    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if line and not line.startswith('---'):
                match = pattern.match(line)
                if match:
                    data.append(match.groupdict())

    # Convert to pandas DataFrame
    df = pd.DataFrame(data)

    if df.empty:
        print("No valid obstacle data found.")
        return df  # Return empty DataFrame if no data is parsed

    # Convert numeric columns to float type
    numeric_cols = [col for col in df.columns if col not in ['obstacle_id']]
    df[numeric_cols] = df[numeric_cols].astype(float)

    return df

