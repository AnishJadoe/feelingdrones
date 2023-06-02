
import numpy as np
import matplotlib.pyplot as plt 
from feelingdrones.engineering.robotics import get_transformations, apply_transform

ORIGIN = np.array([0,0,0,1])
XHAT = np.array([0.5, 0,0,0])
YHAT = np.array([0, 0.5,0,0])

def get_origin_points(base, transformations):

    x_points = []
    y_points = []
    for i in range(4):
        x_points.append(apply_transform(base,transformations,i)[0])
        y_points.append(apply_transform(base,transformations,i)[1])
    return x_points,y_points

def draw_coordinate_frames(ax,transformations, base, xhat,yhat):
    xhat_rotated = xhat
    yhat_rotated = yhat

    # Base of coordinate sytem 
    ax.arrow(*(base)[:-2], dx=xhat_rotated[0], dy=xhat_rotated[1], head_width=0.2, color='r')
    ax.arrow(*(base)[:-2], dx=yhat_rotated[0], dy=yhat_rotated[1], head_width=0.2, color='r')

    # Rotate the x and y unit vectors
    for i in range(4):
        xhat_rotated = apply_transform(xhat, transformations,i)
        yhat_rotated = apply_transform(yhat, transformations,i)
        base_rotated = apply_transform(base,transformations,i)[:-2]
        ax.arrow(*base_rotated, dx=xhat_rotated[0], dy=xhat_rotated[1], head_width=0.2, color='r')
        ax.arrow(*base_rotated, dx=yhat_rotated[0], dy=yhat_rotated[1], head_width=0.2, color='r')
    
    return 


def draw_configuration(ax,q, with_frame=False):

    ax.set_xlim([-15, 15])
    ax.set_ylim([-1, 15])
    ax.set_aspect('equal')
    
    transformations = get_transformations(q)
    points = get_origin_points(ORIGIN,transformations)
    ax.plot(*points)
    if with_frame:
        draw_coordinate_frames(ax,transformations,ORIGIN,XHAT,YHAT)

    return 

    

