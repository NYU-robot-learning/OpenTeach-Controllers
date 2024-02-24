#!/usr/bin/python3

import math
import numpy as np
import os
import rospy
from scipy.spatial.transform import Rotation 
import tf
import time
import yaml

from std_msgs.msg import Float64MultiArray
from deoxys.franka_interface import FrankaInterface

from franka_arm.constants import CONFIG_NUC_ROOT

# REFERENCE_FRAME = 'j2n6s300_link_base'
# END_EFFECTOR_FRAME = 'j2n6s300_link_6'

np.set_printoptions(precision=2, suppress=True)

PUBLISHER_TOPIC = '/franka_driver/hand_gravity_vector'

class Transformer(object):
    def __init__(self, rate = 50):
        try:
            rospy.init_node('transform_listener')
        except:
            pass

        self.Rate = rospy.Rate(rate)

        # Initialize the franka interface
        # Change the yaml file to have a random pub port
        record_file = os.path.join(CONFIG_NUC_ROOT, 'record_deoxys.yml')
        with open(record_file) as f:
            record_yaml = yaml.safe_load(f)
        record_yaml['NUC']['SUB_PORT'] += 2
        record_yaml['NUC']['GRIPPER_SUB_PORT'] += 2
        with open(record_file, 'w') as f:
            yaml.dump(record_yaml, f)

        self.robot_interface = FrankaInterface(
            record_file, use_visualizer=False
        )

        while self.robot_interface.state_buffer_size == 0:
            rospy.logwarn("Robot state not received")
            time.sleep(0.5)

    def record_and_publish(self):
        message = Float64MultiArray() 
        message.data = []
        self.pub = rospy.Publisher(PUBLISHER_TOPIC, Float64MultiArray, queue_size=1)

        while not rospy.is_shutdown():

            H_Ao_R = np.array( # Homo matrix that takes a point in rotated eef to robot's wrist frame
                [[1/np.sqrt(2), -1/np.sqrt(2), 0, 0],
                [-1/np.sqrt(2), -1/np.sqrt(2), 0, 0],
                [0, 0, 1, 0.06], # The height of the allegro mount is 6cm
                [0, 0, 0, 1]])
            
            H_An_Ao = np.array(
                [[0, 1, 0, 0],
                 [1, 0, 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]]
            )

            H_An_R = H_Ao_R @ H_An_Ao
            
            H_R_O  = self.robot_interface.last_eef_pose # Homo matrix that takes a point in robot's frme and moves to the origin's frame
            H_O_A =  np.linalg.inv(H_R_O)@np.linalg.inv(H_An_R)

            G_O = [0, 0, -9.8, 0] # Gravity vector in an arbitrary frame
            G_A = H_O_A @ G_O
            #G_A=np.transpose(G_A)
            g_x, g_y, g_z = G_A[0], G_A[1], G_A[2]
            # rotation_matrix, _ = self.robot_interface.last_eef_rot_and_pos
            # print('rotation_matrix: {}'.format(rotation_matrix))
            # # Again, due to the joint limits of franka we rotate the rotation matrix of 
            # # franka by -45 degrees 
            # rotation_rot = Rotation.from_quat([0, 0, -np.sin(np.pi/8), np.cos(np.pi/8)])  
            # rotation_matrix = np.matmul(rotation_matrix, rotation_rot.as_matrix())
            
            # gravity_vector = [0,0,9.8]
            # rotated_gravity_vector = np.matmul(gravity_vector, rotation_matrix)

            # g_x = rotated_gravity_vector[1]
            # g_y = -rotated_gravity_vector[0]
            # g_z = rotated_gravity_vector[2]

            message.data = [g_x, g_y, g_z]

            # rospy.loginfo(f'*****\nGravity Message: {np.asarray(message.data) / 9.8} \n*****')

            self.pub.publish(message)
            self.Rate.sleep()

            
if __name__ == '__main__':
    t = Transformer()
    
    print("Publishing the dynamic gravity vector for the end effector!")
    t.record_and_publish()