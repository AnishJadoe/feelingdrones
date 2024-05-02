
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

with open ('data/offset.json','r') as f:
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
fig, ax = plt.subplots(1,1, figsize=fig_size)
open_loop_hist = []
# Plot open loop data
for y_offset in y_offsets:
    data = []
    for entry in open_loop_data:
        if entry['y_offset'] == y_offset:
            data.append(entry)
    open_loop_hist.append(calculate_probability(data))

closed_loop_hist = []
# Plot closed loop data
for y_offset in y_offsets:
    data = []
    for entry in closed_loop_data:
        if entry['y_offset'] == y_offset:
            data.append(entry)
    closed_loop_hist.append(calculate_probability(data))

ax.plot(y_offsets,closed_loop_hist,color='orange',lw=3, label="Perching w/ Tactile Feedback")
ax.plot([0,0.05,0.14],closed_loop_hist,color='orange',lw=3)
ax.plot(y_offsets,open_loop_hist,color='blue', lw=3, label="Perching w/o Tactile Feedback")
ax.plot([0,0.05,0.14],open_loop_hist,color='blue',lw=3)

ax.set_ylabel('Probability of Success')
ax.set_xlabel('Y Offset [m]')
# ax.plot([0,0],[0,1.5], color='grey',linewidth=5, label='Perching Object')
ax.legend(fontsize=20)
ax.set_ylim(0,1)
# Add a horizontal bar representing the perching object
# for i in range(2):

#     # Set labels and title

#     ax[i].set_ylabel('X [m]')
#     ax[i].set_xlim((-0.2,0.2))
#     ax[i].set_ylim((-0.2,0.2))
#     ax[i].legend(fontsize=20)

# y_ticks = np.linspace(-0.2, 0.2, 4)
# ax[0].set_yticks(y_ticks)
# ax[1].set_yticks(y_ticks)
# Show the plot
fig.tight_layout()
plt.show()
# plt.savefig('/home/anish/Documents/Thesis/Plots/offset_plot.png', bbox_inches='tight',format='png',dpi=dpi)