from robot import Robot
from dynamixel import Dynamixel
import numpy as np
from get_hardware_offset import ArmOffset

device_name = '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8J0W3F-if00-port0'
baudrate = 57600
# leader_dynamixel = Dynamixel.Config(baudrate=57600, device_name=device_name).instantiate()
# leader = Robot(leader_dynamixel, servo_ids=[1, 2, 3, 4, 5, 6])

# start_pose = leader.read_position()
# print("Follower Arm [Start pos record]: ", start_pose)

# Record Start pose Correction
armoffset = ArmOffset(device_name=device_name, baudrate=baudrate)
correction_joints = armoffset.correction_joints

print("Home pose [Calibration joints]: ", correction_joints)
