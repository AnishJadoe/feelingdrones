
from matplotlib.patches import Circle
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np
import pandas as pd
import json

with open ('mock_logger.json','r') as f:
    json_data = json.load(f)
    
    
# Extract data for open loop and closed loop separately
open_loop_data = [entry for entry in json_data if entry['type'] == 'open_loop:']
closed_loop_data = [entry for entry in json_data if entry['type'] == 'closed_loop:']

# Function to calculate the probability based on perch_success
def calculate_probability(data):
    return np.mean([entry['perch_succesfull'] for entry in data])

y_offsets = [0,-0.1,-0.2]
# Create figure and axis
fig, ax = plt.subplots(1,2)
# Plot open loop data
for y_offset in y_offsets:
    data = []
    for entry in open_loop_data:
        if entry['y_offset'] == y_offset:
            data.append(entry)
    prob = calculate_probability(data)
    rectangle = Circle((0, 0), y_offset,alpha=prob, color='blue', label=f'{prob*100}%')
    ax[0].add_patch(rectangle)

# Plot closed loop data
for y_offset in y_offsets:
    data = []
    for entry in closed_loop_data:
        if entry['y_offset'] == y_offset:
            data.append(entry)
    prob = calculate_probability(data)
    rectangle = Circle((0, 0), y_offset,alpha=prob, color='orange', label=f'{prob*100}%')
    ax[1].add_patch(rectangle)

perching_naming = ['Open Loop', 'Closed Loop']
# Add a horizontal bar representing the perching object
for i in range(2):
    ax[i].plot([0,0],[-0.5,0.5], color='grey',linewidth=5, label='Perching Object')

    # Set labels and title
    ax[i].set_xlabel('Y Offset [m]')
    ax[i].set_ylabel('X [m]')
    ax[i].set_title(f'{perching_naming[i]} Perching \n Probability (Top View)')
    ax[i].set_xlim((-1,1))
    ax[i].set_ylim((-0.5,0.5))
    ax[i].legend(fontsize=6)

# Show the plot
#plt.show()
fig.tight_layout()
plt.savefig('/home/anish/Documents/Thesis/Plots/offset_plot.png', format='png',dpi=2000)