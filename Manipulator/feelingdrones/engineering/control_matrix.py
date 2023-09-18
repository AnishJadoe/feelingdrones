import numpy as np

POSX_T1 = 0.1 # [m]
POSX_T2 = 0.07 # [m]
POSX_T3 = 0.04 # [m]

POSY_T1 = 0.1 # [m]
POSY_T2 = 0.07 # [m]
POSY_T3 = 0.04 # [m]

POSX_T4 = 0.1 # [m]
POSX_T5 = 0.07 # [m]
POSX_T6 = 0.04 # [m]

POSY_T4 = 0.1 # [m]
POSY_T5 = 0.07 # [m]
POSY_T6 = 0.04 # [m]

POSX_T7 = 0.1 # [m]
POSX_T8 = 0.07 # [m]
POSX_T9 = 0.04 # [m]

POSY_T7 = 0.1 # [m]
POSY_T8 = 0.07 # [m]
POSY_T9 = 0.04 # [m]


CASE_1 = np.array([[1,0,0,0,0,0,0,0,0,0,0,0]]).reshape(12,1)
CASE_2 = np.array([[0,1,0,0,0,0,0,0,0,0,0,0]]).reshape(12,1)
CASE_3 = np.array([[0,0,1,0,0,0,0,0,0,0,0,0]]).reshape(12,1)
CASE_4 = np.array([[0,0,0,1,0,0,0,0,0,0,0,0]]).reshape(12,1)
CASE_5 = np.array([[0,0,0,0,1,0,0,0,0,0,0,0]]).reshape(12,1)
CASE_6 = np.array([[0,0,0,0,0,1,0,0,0,0,0,0]]).reshape(12,1)


touch_data = np.array([[0,0,0,0,0,0,0,0,0,0,0,0]]).reshape(12,1)

if touch_data == CASE_1:
    delta_z = 0.5
    delta_x = POSX_T1
    delta_y = POSY_T1

if touch_data == CASE_2:
    delta_z = 0.5
    delta_x = POSX_T4
    delta_y = POSY_T4

if touch_data == CASE_3:
    delta_z = 0.5
    delta_x = POSX_T7
    delta_y = POSY_T7

if touch_data == CASE_4:
    delta_z = 0.5
    delta_x = POSX_T3
    delta_y = POSY_T3

if touch_data == CASE_5:
    delta_z = 0.5
    delta_x = POSX_T6
    delta_y = POSY_T6

if touch_data == CASE_6:
    delta_z = 0.5
    delta_x = POSX_T9
    delta_y = POSY_T9


