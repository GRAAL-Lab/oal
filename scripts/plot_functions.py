import pandas as pd
import matplotlib.pyplot as plt
import ipywidgets as widgets
from IPython.display import display
from obs_functions import *
from trace_functions import *

def plot_init():
    fig, ax = plt.subplots()  # Create a figure and axis for plotting
    ax.set_xlabel('x[m]')
    ax.set_ylabel('y[m]')
    ax.axis('equal')
    plt.subplots_adjust(left=0.3, bottom=0.25)

    ax.set_xlim(-50,100)
    ax.set_ylim(-50,100)

    return fig, ax
