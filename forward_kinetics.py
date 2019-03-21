import numpy as np
import argparse
from urdf_parser_py.urdf import URDF

URDF_PATH = "./locobot_description_v3.urdf"
'''
  Helper Functions
'''

import numpy as np
import argparse
from urdf_parser_py.urdf import URDF
from math import *
F_path = "./locobot_description_v3.urdf"


def getWristPose(joint_angle_list,joint_names, axis_table, joint_table ):


    T = np.eye(4)
    for i,j in enumerate(joint_names):
        xyz = joint_table[j].xyz
        r,p,y = joint_table[j].rpy
        TF_M = np.eye(4)
        TF_M[:3, :3] = E_to_R(r, p, y)
        TF_M[:3, -1] = xyz
        T = T.dot(TF_M)
        x,y,z = axis_table[j]
        new = np.eye(4)
        new[:3,:3] = A_to_R(joint_angle_list[i],x,y,z)
        T = T.dot(new)
    return T

def getWristJacobian(joint_angle, joint_names, joint_table, axis_table, wrist_pose):

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
        T = T.dot(TF_M)
        x,y,z = axis_table[j]
        new[:3,:3] = A_to_R(joint_angle[i],x,y,z)
        T = T.dot(new)

        b1 = T[:3,:3].dot(np.array(axis_table[j]))
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
    return rot_1.dot( rot_2.dot( rot_3))


def get_cuboid_config():
    joint_angles = []

    # assert len(joint_angles) == 5, "Incorrect number of joints specified."
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



def getJointH(theta, axis):
    H = np.eye(4).astype(float)
    return H

def getURDFData():
    robot = URDF.from_xml_file(URDF_PATH)
    joint_name_list = robot.get_chain('arm_base_link', 'gripper_link', links=False)
    axisList = []
    positionList = []
    orientationList = []
    for jnt_name in joint_name_list:
        jnt = robot.joint_map[jnt_name]
        axisList.append(np.array(jnt.axis))
        positionList.append(np.array(jnt.origin.xyz))
        orientationList.append(np.array(jnt.origin.rpy))
    return axisList, positionList, orientationList

def getHmatrixList(position, orientation):
    assert len(position) == len(orientation)
    tfMatrixList = []
    for i in range(len(position)):
        tfMatrixTmp = np.eye(4)
        tfMatrixTmp[:3, -1] = position[i]
        tfMatrixList.append(tfMatrixTmp)
    return tfMatrixList

