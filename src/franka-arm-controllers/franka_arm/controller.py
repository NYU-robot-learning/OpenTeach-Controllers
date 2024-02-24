# This will have our own controller to move the robot - a smaller wrapper on
# top of the deoxys wrapper

import os
import time
import numpy as np
import yaml

from scipy.spatial.transform import Rotation
from deoxys.franka_interface import FrankaInterface
from deoxys.utils import transform_utils
from deoxys.utils.log_utils import get_deoxys_example_logger

from franka_arm.utils import move_joints, get_position_controller_config, get_velocity_controller_config, euler2quat

from .constants import *

class FrankaController:
    def __init__(self, record=False):
        if record:

            # Change the yaml file to have a random pub port
            record_file = os.path.join(CONFIG_ROOT, 'record_deoxys.yml')
            with open(record_file) as f:
                record_yaml = yaml.safe_load(f)
            record_yaml['NUC']['SUB_PORT'] += 2
            record_yaml['NUC']['GRIPPER_SUB_PORT'] += 2
            with open(record_file, 'w') as f:
                yaml.dump(record_yaml, f)

            self.robot_interface = FrankaInterface(
                record_file, use_visualizer=False, control_freq=CONTROL_FREQ,
                state_freq=STATE_FREQ
            )
        else:
            self.robot_interface = FrankaInterface(
                os.path.join(CONFIG_ROOT, 'deoxys.yml'), use_visualizer=False,
                control_freq = CONTROL_FREQ,
                state_freq=STATE_FREQ
            )

        self.controller_type = CONTROLLER_TYPE
        self.position_controller_cfg = get_position_controller_config(
            config_root = CONFIG_ROOT
        )
        self.velocity_controller_cfg = get_velocity_controller_config(
            config_root = CONFIG_ROOT
        )

        self.logger = get_deoxys_example_logger()

        print("robot_interface.state_buffer_size", self.robot_interface.state_buffer_size)

        while self.robot_interface.state_buffer_size == 0:
            self.logger.warn("Robot state not received")
            time.sleep(0.5)

    @property
    def control_freq(self):
        return self.robot_interface._control_freq

    def get_cartesian_position(self):
        current_quat, current_pos = self.robot_interface.last_eef_quat_and_pos

        # Will return 90 degrees negative of it so that the arm end effector will rotate
        # easily
        current_rot, _ = self.robot_interface.last_eef_rot_and_pos
        rotation_rot = Rotation.from_quat([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])

        return current_pos, current_quat

    def get_osc_position(self):
        current_quat, current_pos = self.robot_interface.last_eef_quat_and_pos
        current_axis_angle = transform_utils.quat2axisangle(current_quat)
        return current_pos, current_axis_angle
    
    def get_pose(self):
        pose = self.robot_interface.last_eef_pose
        return pose

    def get_joint_position(self): # NOTE: I am not sure if this is the proper way
        return self.robot_interface.last_q
    
    def joint_movement(self, desired_joint_pos):
        return move_joints(
            robot_interface = self.robot_interface,
            desired_joint_pos = desired_joint_pos, 
            controller_cfg = None # This will automatically be assigned to joint control
        )
    
    def cartesian_control(self, cartesian_pose): # cartesian_pose: (7,) (pos:quat) - pos (3,) translational pose, quat (4,) quaternion

        cartesian_pose = np.array(cartesian_pose, dtype=np.float32)
        target_pos, target_quat = cartesian_pose[:3], cartesian_pose[3:]
        target_mat = transform_utils.pose2mat(pose=(target_pos, target_quat))

        current_quat, current_pos = self.robot_interface.last_eef_quat_and_pos
        current_mat = transform_utils.pose2mat(pose=(current_pos.flatten(), current_quat.flatten()))

        pose_error = transform_utils.get_pose_error(target_pose=target_mat, current_pose=current_mat)

        if np.dot(target_quat, current_quat) < 0.0:
            current_quat = -current_quat
        quat_diff = transform_utils.quat_distance(target_quat, current_quat)
        axis_angle_diff = transform_utils.quat2axisangle(quat_diff)            

        action_pos = pose_error[:3] * TRANSLATIONAL_POSE_VELOCITY_SCALE 
        action_axis_angle = axis_angle_diff.flatten() * ROTATIONAL_POSE_VELOCITY_SCALE
        
        action_pos, _ = transform_utils.clip_translation(action_pos, TRANSLATION_VELOCITY_LIMIT)
        action_axis_angle = np.clip(action_axis_angle, -ROTATION_VELOCITY_LIMIT, ROTATION_VELOCITY_LIMIT)
        action = action_pos.tolist() + action_axis_angle.tolist()

        self.robot_interface.control(
            controller_type=self.controller_type,
            action=action,
            controller_cfg=self.velocity_controller_cfg,
        )


if __name__ == '__main__':
    from holobot.robot.franka import FrankaArm
    franka = FrankaArm()
    FRANKA_HOME_VALUES_CART = [0.65441835, -0.01289619, 0.15598844, -0.27365872, 0.77609015, 0.00515388, 0.56812686]
    franka.move_coords(FRANKA_HOME_VALUES_CART)



