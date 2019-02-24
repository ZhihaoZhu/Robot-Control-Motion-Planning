import numpy as np
import argparse
import rospy
from urdf_parser_py.urdf import URDF
from math import *
F_path = "./locobot_description_v3.urdf"


def getWristPose(joint_angle_list,joint_names, axis_table, joint_table ):
    '''Get the wrist pose for given joint angle configuration.

    joint_angle_list: List of joint angles to get final wrist pose for
    kwargs: Other keyword arguments use as required.

    TODO: You can change the signature of this method to pass in other objects,
    such as the path to the URDF file or a configuration of your URDF file that
    has been read previously into memory.

    Return: List of 16 values which represent the joint wrist pose
    obtained from the End-Effector Transformation matrix using column-major
    ordering.
    '''

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

def getWristJacobian(joint_angle, joint_names, joint_table, axis_table, wrist_pose):
    '''Get the wrist jacobian for given joint angle configuration.

    joint_angle_list: List of joint angles to get final wrist pose for
    kwargs: Other keyword arguments use as required.

    TODO: You can change the signature of this method to pass in other objects,
    such as the wrist pose for this configuration or path to the URDF
    file or a configuration of your URDF file that has been read previously
    into memory.

    Return: List of 16 values which represent the joint wrist pose
    obtained from the End-Effector Transformation matrix using column-major
    ordering.
    '''
    T = np.eye(4)
    position = wrist_pose[:3,-1]
    Jacobian = np.zeros((6,5))
    for i,j in enumerate(joint_names):
        new = np.eye(4)
        TF_M = np.eye(4)

        # Calculate transform matrix
        xyz = joint_table[j].xyz
        r, p, y = joint_table[j].rpy
        TF_M[:3, :3] = E_to_R(r, p, y)
        TF_M[:3, -1] = xyz

        # Apply transform matrix
        T = T @ TF_M
        x,y,z = axis_table[j]
        new[:3,:3] = A_to_R(joint_angle[i],x,y,z)
        T = T @ new

        b1 = T[:3,:3] @ np.array(axis_table[j])
        b2 = np.array(position)-T[:3,-1]
        Jacobian[-3:, i] = b1
        Jacobian[:3,i] = np.cross(b1,b2)

    return Jacobian

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
    jacobian = getWristJacobian(joint_angles, joint_names, joint_table, axis_table, wrist_pose)
    print("Wrist pose: {}".format(np.array_str(np.array(wrist_pose), precision=3)))
    print("Jacobian: {}".format(np.array_str(np.array(jacobian), precision=3)))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
            description='Get wrist pose using forward kinematics')
    parser.add_argument('--joints', type=float, nargs='+', required=True,
                        help='Joint angles to get wrist position for.')
    args = parser.parse_args()

    main()
