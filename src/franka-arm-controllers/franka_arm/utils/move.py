import time
from typing import Union

import numpy as np

from deoxys.utils.config_utils import (get_default_controller_config,
                                       verify_controller_config)

# Joint space motion abstractions

# Taken from https://github.com/UT-Austin-RPL/deoxys_control/

def move_joints(
    robot_interface,
    desired_joint_pos: Union[list, np.ndarray],
    controller_cfg: dict = None,
    timeout=7,
):
    assert type(desired_joint_pos) is list or type(desired_joint_pos) is np.ndarray
    if controller_cfg is None:
        controller_cfg = get_default_controller_config(controller_type="JOINT_POSITION")
    else:
        assert controller_cfg["controller_type"] == "JOINT_POSITION", (
            "This function is only for JOINT POSITION mode. You specified "
            + controller_cfg["controller_type"]
        )
        controller_cfg = verify_controller_config(controller_cfg)
    if type(desired_joint_pos) is list:
        action = desired_joint_pos + [-1.0]
    else:
        action = desired_joint_pos.tolist() + [-1.0]
    start_time = time.time()
    while True:
        if (
            robot_interface.received_states
            and robot_interface.check_nonzero_configuration()
        ):
            if (
                np.max(
                    np.abs(np.array(robot_interface.last_q) - np.array(desired_joint_pos))
                )
                < 1e-3
            ):
                break
        robot_interface.control(
            controller_type="JOINT_POSITION",
            action=action,
            controller_cfg=controller_cfg,
        )
        end_time = time.time()

        # Add timeout
        if end_time - start_time > timeout:
            break
    robot_interface.close()
    return True

