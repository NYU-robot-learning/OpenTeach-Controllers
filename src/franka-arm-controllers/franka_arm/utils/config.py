import os
from easydict import EasyDict
from deoxys.utils.yaml_config import YamlConfig
from deoxys.utils.config_utils import verify_controller_config

def get_velocity_controller_config(config_root):
    controller_cfg = YamlConfig(
        os.path.join(config_root, "osc-pose-controller-velocity.yml")
    ).as_easydict()
    controller_cfg = verify_controller_config(controller_cfg) 

    return controller_cfg

def get_position_controller_config(config_root):
    controller_cfg = YamlConfig(
        os.path.join(config_root, "osc-pose-controller-position.yml")
    ).as_easydict()
    controller_cfg = verify_controller_config(controller_cfg) 

    return controller_cfg