from constants import *
from data_loader import get_data_dict

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
    
def get_tries(df):
    # Initialize variables
    grasp_attempts = 0
    in_grasp_state = False

    # Iterate through the DataFrame
    for _, row in df.iterrows():
        current_state = row['state']

        # Check if the drone is in the GRASP state
        if current_state == 4:
            in_grasp_state = True

        # Check if the drone transitions from GRASP to EVALUATE or from EVALUATE back to GRASP
        elif in_grasp_state and current_state in [5, 8]:
            grasp_attempts += 1
            in_grasp_state = False
        
    return grasp_attempts

path = "/home/anish/Documents/Thesis/Drone/ros2_bag_files/closed_loop_tactile_7_12/test_tactile_4/"
data_dict = get_data_dict(path)

df_command = data_dict[DRONE_STATE]
trajectories = get_tries(df_command)
print(trajectories)