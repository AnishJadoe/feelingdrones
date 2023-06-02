from matplotlib import pyplot as plt
import numpy as np
from feelingdrones.engineering.dynamics import get_gravity_matrix, get_jacobian
from feelingdrones.engineering.robotics import forward_kinematics
from feelingdrones.visual.plotting import draw_configuration

q1_start = np.deg2rad(90) 
q1 = q1_start + np.deg2rad(-40)

q2_start = np.deg2rad(90) 
q2 = q2_start + np.deg2rad(-10)

q3_start = np.deg2rad(90)
q3 = q3_start + np.deg2rad(-60)

q = np.array([q1,q2,q3]).reshape(3,1)

fig, ax = plt.subplots()

draw_configuration(ax,q)

G = get_gravity_matrix(q)
J = get_jacobian(q) # Jacobian

T_04 = forward_kinematics(q)
F = T_04@(np.array([[0],[-2],[0],[1]])) # Force Vector, has to be translated to base frame

K =  (( J.T@(F[:-1]) - G ) / (np.array([[q1 + np.deg2rad(180) - q1_start],[q2],[q3]])) ) * 1000 # Nmm / rad

print(K)
plt.show()