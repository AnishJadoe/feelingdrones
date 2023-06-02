import numpy as np
import matplotlib.pyplot as plt
from feelingdrones.engineering.constants import *
from feelingdrones.visual.plotting import draw_configuration
from feelingdrones.engineering.robotics import get_transformations, apply_transform, get_rotations


RADIUS_PLUM = 2.0 # cm
Y_OFFSET = 4 # cm
X_OFFSET = 1 # cm

L1 = (LENGTH_LINK_1 / 2) * 100 # cm
L2 = (LENGTH_LINK_2 / 2) * 100 # cm
L3 = (LENGTH_LINK_3 / 2) * 100 # cm

center_circle = np.array([X_OFFSET,Y_OFFSET])

q1_start = np.deg2rad(0) 
q1 = q1_start + np.deg2rad(0)

q2_start = np.deg2rad(0) 
q2 = q2_start + np.deg2rad(0)

q3_start = np.deg2rad(0)
q3 = q3_start + np.deg2rad(0)
q = np.array([q1,q2,q3]).reshape(3,1)

original_configuration = q.copy()

p1 = np.array([L1,0,0,1]).reshape(4,1) # in frame 1
p2 = np.array([L2,0,0,1]).reshape(4,1) # in frame 2
p3 = np.array([L3,0,0,1]).reshape(4,1) # in frame 3 

run_q1 = True
run_q2 = True
run_q3 = True

step = 0.1

for i in range(0,1800,1):
 
    q1 += np.deg2rad(step)
    q[0] = q1
    transformations = get_transformations(q)
    new_p1 = apply_transform(p1,transformations,0)
    d1 = np.sqrt( (center_circle[0] - new_p1[0])**2 + (center_circle[1] - new_p1[1])**2 ) 
    if d1 <= RADIUS_PLUM:
        run_q1 = False
        break

for j in range(0,900,1):
   
    q2 += np.deg2rad(step)
    q[1] = q2   
    transformations = get_transformations(q)
    new_p2 = apply_transform(p2,transformations,1)
    d2 = np.sqrt( (center_circle[0] - new_p2[0])**2 + (center_circle[1] - new_p2[1])**2 ) 
    if d2 <= RADIUS_PLUM:
        run_q2 = False
        break

for i in range(0,900,1):

    q3 += np.deg2rad(step)
    q[2] = q3 
    transformations = get_transformations(q)
    new_p3 = apply_transform(p2,transformations,2)
    d3 = np.sqrt( (center_circle[0] - new_p3[0])**2 + (center_circle[1] - new_p3[1])**2 ) 
    if d3 <= RADIUS_PLUM:
        run_q3 = False
        break
                    
transformations = get_transformations(q)
p1 = apply_transform(p1,transformations,0)
p2 = apply_transform(p2,transformations,1)
p3 = apply_transform(p3,transformations,2)
# Set up the plot
fig, ax = plt.subplots()

plum = plt.Circle((X_OFFSET, Y_OFFSET), radius=RADIUS_PLUM, color='r')
ax.add_patch(plum)


draw_configuration(ax,q)

ax.scatter(p1[0],p1[1],s=20)
ax.scatter(p2[0],p2[1],s=20)
ax.scatter(p3[0],p3[1],s=20)


plt.grid()
plt.show()
print(np.rad2deg(q))