# Import system libraries
import argparse
import os
import sys

# Modify the following lines if you have problems importing the V-REP utilities
cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(os.path.join(cwd, 'lib'))
sys.path.append(os.path.join(cwd, 'utilities'))

# Import application libraries
import numpy as np
import vrep_utils as vu
from PRM import *
from Dijkstra import *

# Import any other libraries you might want to use ############################
import matplotlib.pyplot as plt


###############################################################################

class ArmController:
    def __init__(self):
        # Fill out this method ##################################
        # Define any variables you may need here for feedback control
        #########################################################
        # Do not modify the following variables
        self.err_cur = np.zeros(7)
        self.err_acc = np.zeros(7)
        self.kp = 10.0
        self.ki = 0.3
        self.kd = 0.2
        self.rate = 1e-3
        self.num = 0
        self.history = {'timestamp': [],
                        'joint_feedback': [],
                        'joint_target': [],
                        'ctrl_commands': []}
        self._target_joint_positions = None

    def set_target_joint_positions(self, target_joint_positions):
        assert len(target_joint_positions) == vu.N_ARM_JOINTS, \
            'Expected target joint positions to be length {}, but it was length {} instead.'.format(
                len(target_joint_positions), vu.N_ARM_JOINTS)
        self._target_joint_positions = target_joint_positions

    def calculate_commands_from_feedback(self, timestamp, sensed_joint_positions):
        assert self._target_joint_positions, \
            'Expected target joint positions to be set, but it was not.'

        # Fill out this method ##################################
        # Using the input joint feedback, and the known target joint positions,
        # calculate the joint commands necessary to drive the system towards
        # the target joint positions.
        err = np.array(self._target_joint_positions) - np.array(sensed_joint_positions)
        self.err_cur = err
        self.err_acc += self.err_cur
        if len(self.history['timestamp']) != 0:
            err_d = err / (timestamp - self.history['timestamp'][-1])
        else:
            err_d = err / timestamp
        ctrl_commands = self.kp * err + self.kd * err_d + self.ki * self.err_acc

        # ...
        #########################################################

        # Do not modify the following variables
        # append time history
        self.history['timestamp'].append(timestamp)
        self.history['joint_feedback'].append(sensed_joint_positions)
        self.history['joint_target'].append(self._target_joint_positions)
        self.history['ctrl_commands'].append(ctrl_commands)
        return ctrl_commands

    def has_stably_converged_to_target(self):
        # Fill out this method ##################################
        if np.sum(self.err_cur) <= self.rate:
            self.num += 1
            if self.num > 150:
                self.num = 0
                return True
            else:
                return False
        else:
            self.num = 0

        # ...
        #########################################################
'''
    Global variables
'''
link_cuboid_spec = []
obstacle_cuboid_spec = []


def main(args):
    global link_cuboid_spec
    global obstacle_cuboid_spec

    # Connect to V-REP
    print('Connecting to V-REP...')
    clientID = vu.connect_to_vrep()
    print('Connected.')

    # Reset simulation in case something was running
    vu.reset_sim(clientID)

    # Initial control inputs are zero
    vu.set_arm_joint_target_velocities(clientID, np.zeros(vu.N_ARM_JOINTS))

    # Despite the name, this sets the maximum allowable joint force
    vu.set_arm_joint_forces(clientID, 50. * np.ones(vu.N_ARM_JOINTS))

    # One step to process the above settings
    vu.step_sim(clientID)

    deg_to_rad = np.pi / 180.

    link_cuboid_list = ("arm_base_link_joint_collision_cuboid", "shoulder_link_collision_cuboid",
                   "elbow_link_collision_cuboid","forearm_link_collision_cuboid",
                   "wrist_link_collision_cuboid","gripper_link_collision_cuboid",
                   "finger_r_collision_cuboid","finger_l_collision_cuboid")
    for link_cuboid in link_cuboid_list:
        d = vu.get_handle_by_name(clientID, link_cuboid)
        link_spec = {}
        link_spec["Origin"] = vu.get_object_position(clientID, d)
        link_spec["Orientation"] = vu.get_object_orientation(clientID, d)
        link_spec["Dimension"] = vu.get_object_bounding_box(clientID, d)
        link_cuboid_spec.append(link_spec)


    obstacle_cuboid_list = ("cuboid_0", "cuboid_1",
                        "cuboid_2", "cuboid_3",
                        "cuboid_4", "cuboid_5")
    for obstacle_cuboid in obstacle_cuboid_list:
        d = vu.get_handle_by_name(clientID, obstacle_cuboid)
        obstacle_spec = {}
        obstacle_spec["Origin"].append(vu.get_object_position(clientID, d))
        obstacle_spec["Orientation"].append(vu.get_object_orientation(clientID, d))
        obstacle_spec["Dimension"].append(vu.get_object_bounding_box(clientID, d))
        obstacle_cuboid_spec.append(obstacle_spec)




    # n_samples = 100
    # K = 3
    # samples, edges, edge_length = PRM(n_samples, K)
    # joint_targets = Dijkstra(samples, edges, edge_length)





    # joint_targets = [[0.,
    #                   0.,
    #                   0.,
    #                   0.,
    #                   0.,
    #                   - 0.07,
    #                   0.07], \
    #                  [-45. * deg_to_rad,
    #                   -15. * deg_to_rad,
    #                   20. * deg_to_rad,
    #                   15. * deg_to_rad,
    #                   -75. * deg_to_rad,
    #                   - 0.03,
    #                   0.03], \
    #                  [30. * deg_to_rad,
    #                   60. * deg_to_rad,
    #                   -65. * deg_to_rad,
    #                   45. * deg_to_rad,
    #                   0. * deg_to_rad,
    #                   - 0.05,
    #                   0.05]]
    #


    # Instantiate controller



    # controller = ArmController()
    #
    # # Iterate through target joint positions
    # for target in joint_targets:
    #
    #     # Set new target position
    #     controller.set_target_joint_positions(target)
    #
    #     steady_state_reached = False
    #     while not steady_state_reached:
    #         timestamp = vu.get_sim_time_seconds(clientID)
    #         print('Simulation time: {} sec'.format(timestamp))
    #
    #         # Get current joint positions
    #         sensed_joint_positions = vu.get_arm_joint_positions(clientID)
    #
    #         # Calculate commands
    #         commands = controller.calculate_commands_from_feedback(timestamp, sensed_joint_positions)
    #
    #         # Send commands to V-REP
    #         vu.set_arm_joint_target_velocities(clientID, commands)
    #
    #         # Print current joint positions (comment out if you'd like)
    #         print(sensed_joint_positions)
    #         vu.step_sim(clientID, 1)
    #
    #         # Determine if we've met the condition to move on to the next point
    #         steady_state_reached = controller.has_stably_converged_to_target()
    #
    # vu.stop_sim(clientID)

    # Post simulation cleanup -- save results to a pickle, plot time histories, etc #####
    # Fill this out here (optional) or in your own script
    # If you use a separate script, don't forget to include it in the deliverables
    # ...
    #####################################################################################



    # plt.figure()
    #
    # for i in range(7):
    #     plt.subplot(2, 4, i + 1)
    #     plt.title('Joint %d' % (i+1))
    #     plt.xlabel('Time')
    #     plt.ylabel('Joint angle')
    #     a1 = np.array(controller.history['joint_feedback'])[:, i]
    #     a2 = np.array(controller.history['joint_target'])[:, i]
    #     plt.plot(0.05 * np.arange(len(a1)), a1)
    #     plt.plot(0.05 * np.arange(len(a2)), a2)
    # plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    args = parser.parse_args()
    main(args)
