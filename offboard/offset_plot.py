
from matplotlib import font_manager
from matplotlib.patches import Circle, Rectangle
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np
import pandas as pd
import json
# Specify the font file path
font_path = '/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf'

# Register the font
font_manager.fontManager.addfont(font_path)

plt.rcParams['font.size'] = 35
plt.rcParams['font.family'] = 'Times New Roman'
fig_size = (20,10)
dpi = 250

with open ('offset.json','r') as f:
    json_data = json.load(f)
    
    
# Extract data for open loop and closed loop separately
open_loop_data = [entry for entry in json_data if entry['type'] == 'open_loop:']
closed_loop_data = [entry for entry in json_data if entry['type'] == 'closed_loop:']

# Function to calculate the probability based on perch_success
def calculate_probability(data):
    return np.mean([entry['perch_succesfull'] for entry in data])
RECT_HEIGHT = 0.1
y_offsets = [0,-0.05,-0.14]
# Create figure and axis
fig, ax = plt.subplots(1,2, figsize=fig_size)
# Plot open loop data
for y_offset in y_offsets:
    data = []
    for entry in open_loop_data:
        if entry['y_offset'] == y_offset:
            data.append(entry)
    prob = calculate_probability(data)
    if y_offset == 0:
        y_offset = 0.01
    rectangle = Rectangle((0, -0.05), width=y_offset,height=RECT_HEIGHT,alpha=prob, color='blue', label=f'{prob*100}%')
    ax[0].add_patch(rectangle)
    rectangle = Rectangle((0, -0.05), width=-y_offset,height=RECT_HEIGHT,alpha=prob, color='blue')
    ax[0].add_patch(rectangle)
# Plot closed loop data
for y_offset in y_offsets:
    data = []
    for entry in closed_loop_data:
        if entry['y_offset'] == y_offset:
            data.append(entry)
    prob = calculate_probability(data)
    if y_offset == 0:
        y_offset = 0.01
    rectangle = Rectangle((0, -0.05), width=y_offset,height=RECT_HEIGHT,alpha=prob, color='orange', label=f'{prob*100}%')
    ax[1].add_patch(rectangle)
    rectangle = Rectangle((0, -0.05), width=-y_offset,height=RECT_HEIGHT,alpha=prob, color='orange')
    ax[1].add_patch(rectangle)

perching_naming = ['Open Loop', 'Closed Loop']
# Add a horizontal bar representing the perching object
for i in range(2):
    ax[i].plot([0,0],[-0.5,0.5], color='grey',linewidth=5, label='Perching Object')

    # Set labels and title
    ax[i].set_xlabel('Y Offset [m]')
    ax[i].set_ylabel('X [m]')
    ax[i].set_xlim((-0.2,0.2))
    ax[i].set_ylim((-0.2,0.2))
    ax[i].legend(fontsize=20)

# y_ticks = np.linspace(-0.2, 0.2, 4)
# ax[0].set_yticks(y_ticks)
# ax[1].set_yticks(y_ticks)
# Show the plot
fig.tight_layout()
plt.savefig('/home/anish/Documents/Thesis/Plots/offset_plot.png', bbox_inches='tight',format='png',dpi=dpi)