import numpy as np
import argparse
import rospy
from urdf_parser_py.urdf import URDF
from math import *


def getWristPose(joint_angle_list,joint_names, axis_table, joint_table ):

    T = np.eye(4)
    for i,j in enumerate(joint_names):
        xyz = joint_table[j].xyz
        r,p,y = joint_table[j].rpy
        TF_M = np.eye(4)
        TF_M[:3, :3] = E_to_R(r, p, y)
        TF_M[:3, -1] = xyz
        T = T @ TF_M
        x,y,z = axis_table[j]
        new = np.eye(4)
        new[:3,:3] = A_to_R(joint_angle_list[i],x,y,z)
        T = T @ new
    return T


def A_to_R(angle, x, y, z):
    R = np.array([[cos(angle)+(x**2)*(1-cos(angle)),x*y*(1-cos(angle))-z*sin(angle),x*z*(1-cos(angle))+y*sin(angle)],
                         [y*x*(1-cos(angle))+z*sin(angle),cos(angle)+(y**2)*(1-cos(angle)),y*z*(1-cos(angle))-x*sin(angle)],
                         [z*x*(1-cos(angle))-y*sin(angle),z*y*(1-cos(angle))+x*sin(angle),cos(angle)+(z**2)*(1-cos(angle))]])
    return R

def E_to_R(r,p,y):
    rot_1 = np.array([[cos(y),sin(y),0],
                        [sin(y),cos(y),0],
                        [0,0,1]])

    rot_2 = np.array([[cos(p),0,sin(p)],
                          [0,1,0],
                          [-sin(p),0,cos(p)]])

    rot_3 = np.array([[1,0,0],
                         [0,cos(r),-sin(r)],
                         [0,sin(r),cos(r)]])
    return rot_1 @ rot_2 @ rot_3



def main(args):
    joint_angles = args.joints

    assert len(joint_angles) == 5, "Incorrect number of joints specified."
    joint_table, axis_table = {}, {}
    robot = URDF.from_xml_file(F_path)
    joint_names = robot.get_chain('arm_base_link','gripper_link',links=False)
    for i in joint_names:
        joint_info = robot.joint_map[i]
        joint_table[i] = joint_info.origin
        axis_table[i] = joint_info.axis

    wrist_pose = getWristPose(joint_angles, joint_names, axis_table, joint_table)
    print("Wrist pose: {}".format(np.array_str(np.array(wrist_pose), precision=3)))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
            description='Get wrist pose using forward kinematics')
    parser.add_argument('--joints', type=float, nargs='+', required=True,
                        help='Joint angles to get wrist position for.')
    args = parser.parse_args()

    main()
