import sympy as sp
import numpy as np
import pandas as pd
import csv

class ForwardKinematicsUR5e:
    """
    Class to calculate the forward kinematics of a 6-DOF robot arm.
    """

    def __init__(self):
        pass

    def Transformation_matrix(self, θdeg, αdeg, a, d):
        αrad = αdeg
        θrad = θdeg

        return np.array([
            [np.cos(θrad), -np.sin(θrad) * np.cos(αrad), np.sin(θrad) * np.sin(αrad), a * np.cos(θrad)],
            [np.sin(θrad), np.cos(θrad) * np.cos(αrad), -np.cos(θrad) * np.sin(αrad), a * np.sin(θrad)],
            [0, np.sin(αrad), np.cos(αrad), d],
            [0, 0, 0, 1]
        ])
    
    # UR3e
    def T1 (self, θdeg1):
        return self.Transformation_matrix(θdeg1, (-np.pi/2), 0, 151.85)
    def T2 (self, θdeg2):
        return self.Transformation_matrix((θdeg2 + np.pi/2), np.pi, -243.55, 0)
    def T3 (self, θdeg3):
        return self.Transformation_matrix(θdeg3, 0, -213.2, 0)
    def T4 (self, θdeg4):
        return self.Transformation_matrix((θdeg4 + np.pi/2), -np.pi/2, 0, 131.05)
    def T5 (self, θdeg5):
        return self.Transformation_matrix(θdeg5,(-np.pi/2), 0, 85.35)
    def T6 (self, θdeg6):
        return self.Transformation_matrix(θdeg6, 0, 0, 92.1)

    # UR10
    # def T1 (self, θdeg1):
    #     return self.Transformation_matrix(θdeg1, (-np.pi/2), 0, 128)
    # def T2 (self, θdeg2):
    #     return self.Transformation_matrix((θdeg2 + np.pi/2), np.pi, -612.7, 0)
    # def T3 (self, θdeg3):
    #     return self.Transformation_matrix(θdeg3, 0, -571.6, 0)
    # def T4 (self, θdeg4):
    #     return self.Transformation_matrix((θdeg4 + np.pi/2), -np.pi/2, 0, -163.9)
    # def T5 (self, θdeg5):
    #     return self.Transformation_matrix(θdeg5,(-np.pi/2), 0, 115.7)
    # def T6 (self, θdeg6):
    #     return self.Transformation_matrix(θdeg6, 0, 0, 92.2)


    # # gello
    # def T1 (self, θdeg1):
    #     return self.Transformation_matrix(θdeg1, (-90), 0, 59)
    # def T2 (self, θdeg2):
    #     return self.Transformation_matrix((θdeg2 - 90), 180, 112.7, 76)
    # def T3 (self, θdeg3):
    #     return self.Transformation_matrix(θdeg3, 180, 107.4, 76)
    # def T4 (self, θdeg4):
    #     return self.Transformation_matrix((θdeg4 + 90), -90, 0, 47.728)
    # def T5 (self, θdeg5):
    #     return self.Transformation_matrix(θdeg5,(-90), 0, 45.95)
    # def T6 (self, θdeg6): 
    #     return self.Transformation_matrix(θdeg6, 0, 0, 58.65)

    def get_transformation_matrix(self, θdeg1=0, θdeg2=0, θdeg3=0, θdeg4=0, θdeg5=0, θdeg6=0):
        T = self.T1(θdeg1) @ self.T2(θdeg2) @ self.T3(θdeg3) @ self.T4(θdeg4) @ self.T5(θdeg5) @ self.T6(θdeg6)
        return T

    def end_eff_pos(self, T):
        R = T[:3, :3]
        x, y, z = T[:3, 3]
        yaw = np.arctan2(R[1, 0], R[0, 0])
        pitch = np.arcsin(-R[2, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])
        x = x/1000
        y = y/1000
        z = z/1000
        return x, y, z, roll, pitch, yaw

    def convert_to_quaternions(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return w, x, y, z

if __name__=="__main__":
    fk = ForwardKinematicsUR5e()

    read_file = 'csv/output18.csv'
    write_file = 'csv/original.csv'
    # write_file = 'csv/outputcompare.csv'

    reader = pd.read_csv(read_file)
    joint_angles = reader.iloc[:, :6].values
    end_effector_pos = reader.iloc[:, 7:13].values

    results = []

    for i in range(len(joint_angles)):
        # print(joint_angles[i])
        T = fk.get_transformation_matrix(*joint_angles[i])
        x, y, z, roll, pitch, yaw = fk.end_eff_pos(T)
        
        xdiff = end_effector_pos[i, 0] - x
        ydiff = end_effector_pos[i, 1] - y
        zdiff = end_effector_pos[i, 2] - z
        rolldiff = end_effector_pos[i, 3] - roll
        pitchdiff = end_effector_pos[i, 4] - pitch
        yawdiff = end_effector_pos[i, 5] - yaw

        # results.append([xdiff, ydiff, zdiff, rolldiff, pitchdiff, yawdiff])
        results.append([x, y, z, roll, pitch, yaw])

    with open(write_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        # writer.writerow(['xdiff', 'ydiff', 'zdiff', 'rolldiff', 'pitchdiff', 'yawdiff'])
        # writer.writerows(results)

        writer.writerow(['x', 'y', 'z', 'roll', 'pitch', 'yaw'])
        writer.writerows(results)




# if __name__=="__main__":
#     np.set_printoptions(suppress=True, precision=2) 
#     fk = ForwardKinematicsUR5e()
#     T = fk.get_transformation_matrix(0, 0, 0, 0, 0, -1.57)
#     print(T)
#     x, y, z, roll, pitch, yaw = fk.end_eff_pos(T)
#     print("Roll: ", roll, "Pitch: ", pitch, "Yaw: ", yaw)
#     print("X: ", x, "Y: ", y, "Z: ", z)
#     w, x, y, z = fk.convert_to_quaternions(roll, pitch, yaw)
#     print("Quaternion: ", w, x, y, z)