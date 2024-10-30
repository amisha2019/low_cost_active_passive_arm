from ur_ikfast.ur_ikfast import ur_kinematics
import pandas as pd
from scipy.spatial.transform import Rotation
import csv
import numpy as np

if __name__=="__main__":
    ur3e_arm = ur_kinematics.URKinematics('ur3e')

    read_file = 'csv/output18.csv'
    # write_file = 'csv/original.csv'
    write_file = 'csv/outputcompare.csv'

    reader = pd.read_csv(read_file)
    joint_angles = reader.iloc[:, :6].values
    end_effector_pos = reader.iloc[:, 7:13].values

    results = []

    for i in range(len(joint_angles)):
        # print("joint angles", joint_angles[i])

        pose_quat = ur3e_arm.forward(joint_angles[i])

        # print("forward() quaternion \n", pose_quat)

        x, y, z, wx, wy, wz, w = [pose for pose in pose_quat]

        x = -x
        y = -y

        pitch, yaw, roll = Rotation.from_quat([wx, wy, wz, w]).as_euler('xyz', degrees=True)

        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)

        print("forward() quaternion \n", x, y, z, roll, pitch, yaw)

        xdiff = end_effector_pos[i, 0] - x
        ydiff = end_effector_pos[i, 1] - y
        zdiff = end_effector_pos[i, 2] - z
        rolldiff = end_effector_pos[i, 3] - roll
        pitchdiff = end_effector_pos[i, 4] - pitch
        yawdiff = end_effector_pos[i, 5] - yaw

        results.append([xdiff, ydiff, zdiff, rolldiff, pitchdiff, yawdiff])
        # results.append([x, y, z, roll, pitch, yaw])

    with open(write_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['xdiff', 'ydiff', 'zdiff', 'rolldiff', 'pitchdiff', 'yawdiff'])
        writer.writerows(results)

        # writer.writerow(['x', 'y', 'z', 'roll', 'pitch', 'yaw'])
        # writer.writerows(results)
