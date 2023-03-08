import numpy as np
import matplotlib.pyplot as plt



l_1 = 10
l_2 = 5
l_3 = 5

theta_1 = np.deg2rad(0)
theta_2 = np.deg2rad(-10)
theta_3 = np.deg2rad(0)

T_0_1 = np.array(
    [
        [np.cos(theta_1), -np.sin(theta_1), l_1 * np.cos(theta_1)],
        [np.sin(theta_1), np.cos(theta_1), l_1 * np.sin(theta_1)],
        [0, 0, 1],
    ]
)
T_1_2 = np.array(
    [
        [np.cos(theta_2), -np.sin(theta_2), l_2 * np.cos(theta_2)],
        [np.sin(theta_2), np.cos(theta_2), l_2 * np.sin(theta_2)],
        [0, 0, 1],
    ]
)
T_2_3 = np.array(
    [
        [np.cos(theta_3), -np.sin(theta_3), l_3 * np.cos(theta_3)],
        [np.sin(theta_3), np.cos(theta_3), l_3 * np.sin(theta_3)],
        [0, 0, 1],
    ]
)

array_1 = np.array([0, 0, 1]).reshape(3, 1)
array_2 = T_0_1.dot(array_1)
array_3 = T_1_2.dot(array_2)
array_4 = T_2_3.dot(array_3)

point_1 = tuple([float(point) for point in array_1[:2]])
point_2 = tuple([float(point) for point in array_2[:2]])
point_3 = tuple([float(point) for point in array_3[:2]])
point_4 = tuple([float(point) for point in array_4[:2]])

plt.figure()
plt.plot([point_1[1], point_2[1]], [point_1[0], point_2[0]], "r")
plt.plot([point_2[1], point_3[1]], [point_2[0], point_3[0]], "g")
plt.plot([point_3[1], point_4[1]], [point_3[0], point_4[0]], "b")

plt.show()
