# Passive Arm ID: /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISI2Y-if00-port0
# Active Arm ID: /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8J0W3F-if00-port0

from robot import Robot
from dynamixel import Dynamixel
import numpy as np
import time
import mujoco.viewer
from simulation.interface import SimulatedRobot
import threading
from get_hardware_offset import ArmOffset

device_name = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISI2Y-if00-port0'
baudrate = 57600
correction_joints = ArmOffset(device_name=device_name, baudrate=baudrate).correction_joints

def read_leader_position():
    global target_pos
    while True:
        target_pos = np.array(leader.read_position())
        target_pos = (target_pos / 2048 - 1) * 3.14
        target_pos[1] = -target_pos[1]
        target_pos[3] = -target_pos[3]
        target_pos[4] = -target_pos[4]
        target_pos = target_pos - correction_joints
        print(target_pos)
leader_dynamixel = Dynamixel.Config(baudrate=baudrate, device_name=device_name).instantiate()
leader = Robot(leader_dynamixel, servo_ids=[1, 2, 3, 4, 5, 6])
# leader.set_trigger_torque()

m = mujoco.MjModel.from_xml_path('simulation/gello_active/scene.xml')
d = mujoco.MjData(m)

r = SimulatedRobot(m, d)
print(d.ctrl[:])
target_pos = np.zeros(5)

# Start the thread for reading leader position
leader_thread = threading.Thread(target=read_leader_position)
leader_thread.daemon = True
leader_thread.start()

with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while viewer.is_running():
        # Use the latest target_pos
        step_start = time.time()
        target_pos_local = target_pos.copy()
        # print(f'target pos copy {time.time() - step_start}')
        r.set_target_pos(target_pos_local)
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
