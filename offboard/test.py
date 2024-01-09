import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Create some sample data
x = np.linspace(0, 10, 100)
y = np.sin(x)

# Create a gradient background
background_data = np.linspace(0, 1, len(x)).reshape(1, -1)

# Set up the figure and axis
fig, ax = plt.subplots()
line, = ax.plot(x, y, label='Plot')
background = ax.imshow(background_data, cmap='viridis', aspect='auto', extent=[x.min(), x.max(), -1, 1], alpha=0.3)

# Update function for animation
def update(frame):
    # Update the plot data
    line.set_ydata(np.sin(x + frame * 0.1))

    return line, background

# Create the animation
anim = FuncAnimation(fig, update, frames=100, interval=100)

# Display the animation
plt.show()
