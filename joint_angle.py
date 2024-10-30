from robot import Robot
from dynamixel import Dynamixel
import numpy as np
import time
import mujoco.viewer
from simulation.interface import SimulatedRobot
import threading
import csv
import pandas as pd

file_name = 'test_csv.csv'

with open(file_name, mode='r') as file:
    df = pd.read_csv(file_name, usecols = range(0, 6), engine="python").astype(np.float64)
    joint_angles = df.iloc[:].values
    
def set_joint_pose_from_csv():
    global target_pos
    for joint_angle in joint_angles:
        target_pos = joint_angle
        time.sleep(0.1)


m = mujoco.MjModel.from_xml_path('simulation/gello_active/scene.xml')
d = mujoco.MjData(m)

r = SimulatedRobot(m, d)

# Start the thread for reading leader position and setting the target position
leader_thread = threading.Thread(target=set_joint_pose_from_csv)
leader_thread.daemon = True
leader_thread.start()

with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while viewer.is_running():
        
        # Use the latest target_pos
        step_start = time.time()
        # target_pos_local = target_pos.copy()
        # print(f'target pos copy {time.time() - step_start}')for

        r.set_target_pos(target_pos)
        # print(f'set targtee pos copy {time.time() - step_start}')
        mujoco.mj_step(m, d)
        # print(f'mjstep {time.time() - step_start}')
        viewer.sync()
        # print(f'viewer sync {time.time() - step_start}')

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        # print(f'time until next step {time_until_next_step}')
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
