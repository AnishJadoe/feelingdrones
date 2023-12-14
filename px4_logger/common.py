from matplotlib import pyplot as plt
from matplotlib.offsetbox import OffsetImage
from constants import *
def get_trajectories(df,trajectories, trajectory_start):
    
    
    df_trajectory = df[trajectory_start:]
    previous_timestamp = trajectory_start
    
    if df_trajectory['state'].loc[trajectory_start] == LAND:
        return trajectories
    
    for current_timestamp in df_trajectory.index:
        if (df_trajectory.loc[current_timestamp]['state'] - df_trajectory.loc[previous_timestamp]['state']) == (EVALUATE - GRASP):
            trajectory_end = previous_timestamp
            if trajectory_end - trajectory_start >= 5:
                trajectories.append((trajectory_start,trajectory_end))
            break
        previous_timestamp = current_timestamp
    trajectories = get_trajectories(df_trajectory, trajectories, current_timestamp)
    
    return trajectories

def getImage(path,zoom=0.07):
   return OffsetImage(plt.imread(path, format="png"), zoom=zoom)