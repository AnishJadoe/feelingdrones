import pandas as pd
import numpy as np

LOGGER_FOLDER = '/home/anish/Desktop/log/'
vehicle_local_position = pd.read_csv(LOGGER_FOLDER + '12_32_00_vehicle_local_position_0.csv')
vehicle_trajectory_setpoint = pd.read_csv(LOGGER_FOLDER + '12_32_00_trajectory_setpoint_0.csv')

print(vehicle_local_position)
