from robot import Robot
from dynamixel import Dynamixel
from get_hardware_offset import ArmOffset
import numpy as np
import time

leader_device= "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISI2Y-if00-port0"
follower_device = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8J0W3F-if00-port0"

## Follower Arm Start pos correction
home_pos_correction = np.array([-1.57, 1.57,  3.14, -1.57, -3.14,  0])

## ----- Activate Leader Arm ---------
# leader_armoffset = ArmOffset(device_name=leader_device, baudrate=57600)
# leader_correction_joints = leader_armoffset.correction_joints
# leader_dynamixel = Dynamixel.Config(baudrate=57600, device_name=leader_device).instantiate()
# for i in range(1, 7):
#     leader_dynamixel.set_velocity_limit(i, 10)

## ----- Activate Follower Arm ---------
# follower_armoffset = ArmOffset(device_name=follower_device, baudrate=57600)
# follower_correction_joints = follower_armoffset.correction_joints
follower_dynamixel = Dynamixel.Config(baudrate=57600, device_name=follower_device).instantiate()

# leader = Robot(leader_dynamixel, servo_ids=[1, 2, 3, 4, 5, 6])
for i in range(1, 7):
    follower_dynamixel.set_velocity_limit(i, 10)
follower = Robot(follower_dynamixel, servo_ids=[1, 2, 3, 4, 5, 6])


def resol_2_rad(angles_resol):
    angles_rad = (angles_resol / 2048 - 1) * 3.14
    angles_rad[1] = -angles_rad[1]
    angles_rad[3] = -angles_rad[3]
    angles_rad[4] = -angles_rad[4]
    return angles_rad

def rad_2_resol(angles_rad):
    angles_rad[1] = -angles_rad[1]
    angles_rad[3] = -angles_rad[3]
    angles_rad[4] = -angles_rad[4]
    angles_resol = (angles_rad / 3.14 + 1) * 2048
    return angles_resol

def get_angles_to_publish_rad(angles_rad):
    corrected_joint_rad = angles_rad + home_pos_correction
    corrected_joint_rad[1] = -corrected_joint_rad[1]
    corrected_joint_rad[3] = -corrected_joint_rad[3]
    corrected_joint_rad[4] = -corrected_joint_rad[4]
    return corrected_joint_rad

# def read_leader_position():
#     global leader_joints
#     leader_joints = leader.read_position()
#     leader_joints = np.array(leader_joints)        
#     leader_joints_rad = resol_2_rad(leader_joints)
#     print("Actual resol: ", leader_joints)
#     print("Actual angle: ", leader_joints_rad)
#     leader_joints_rad = (leader_joints_rad - leader_correction_joints)
#     # leader_joints_rad_ = [math.fmod(x, 3.14) for x in leader_joints_rad]
#     print("Actual corrected angle: ", leader_joints_rad)

#     return leader_joints_rad

def set_follower_position(joints_rad):
    # Correct the radians of the follower
    pass

def go_to_start_position():
    start_pos = np.array([-1.57, -1.57, -2, -1.57, 1.57, 1.57])

    # Correct the radians of the leader
    corrected_joint_rad = get_angles_to_publish_rad(start_pos)
    follower_joints_resol = np.array(rad_2_resol(corrected_joint_rad), dtype=int)
    
    print("reading: ", follower.read_position())
    # leader_joints_rad = resol_2_rad(np.array(follower.read_position()))
    # print("Reding rad: ", leader_joints_rad)

    print("Joints to publish rad: ", corrected_joint_rad)
    print("Joints to publish: ", (follower_joints_resol))
    
    #### ____DANGER ZONE____ ####
    ## Read Current Position - Follower - [set Home position]
    # curr_pos = follower.read_position()
    # print("Current Pos: ", curr_pos)
    # print("Going to Home Pose: ")    
    # for i in range(0, len(curr_pos)):
    #     print(f"Curr_Pose_joint{i+1}: {curr_pos[i]}")
    #     curr_pos[i] = follower_joints_resol[i]  
    #     print(f"Set_Pose_joint{i+1}: {curr_pos[i]}")
    #     follower.set_goal_pos(curr_pos)
    #     print(f"{i+1}: done")
    #     time.sleep(2)        
    # print("final Pose")
    # print(resol_2_rad(np.array(follower.read_position())))
    # follower._disable_torque()


while True:
    # follower.set_goal_pos(leader.read_position())
    # Get Leader Arm pos in radians
    # joints_rad = read_leader_position()
    # print("Passive Arm: ", joints_rad)

    # Set Follower Arm pos in radians
    go_to_start_position()
    break
    # read_leader_position()
    # read_leader_position()
    # print(leader.read_position())