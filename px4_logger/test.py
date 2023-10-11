import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

DATA_FOLDER = '~/dev/working/feelingdrones/px4_logger/Data/0919/'
DATA_FILE = '13_00_24'
vehicle_local_position = pd.read_csv(DATA_FOLDER + f'/{DATA_FILE}/' + f'{DATA_FILE}_vehicle_local_position_0.csv')
vehicle_trajectory_setpoint = pd.read_csv(DATA_FOLDER + f'/{DATA_FILE}/' + f'{DATA_FILE}_trajectory_setpoint_0.csv')


fig,axs = plt.subplots(2,2)

axs[0,0].plot(vehicle_local_position['x'], label='est_x')
axs[0,0].plot(vehicle_trajectory_setpoint['position[0]'], label='ref_x')
axs[0,0].set_title('x')

axs[1,0].plot(vehicle_local_position['y'], label='est_y')
axs[1,0].plot(vehicle_trajectory_setpoint['position[1]'], label='ref_y')
axs[1,0].set_title('y')

axs[0,1].plot(vehicle_local_position['z'], label='est_z')
axs[0,1].plot(vehicle_trajectory_setpoint['position[2]'], label='ref_z')
axs[0,1].set_title('z')

fig.legend()
plt.show()

