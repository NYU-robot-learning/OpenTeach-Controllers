# These could be changed accordingly

CONTROLLER_TYPE = "OSC_POSE"
CONFIG_ROOT = '/home/aadhithya/Workspace/dexterous-arm-controllers/src/franka-arm-controllers/franka_arm/configs'
CONFIG_NUC_ROOT = '/home/grail/workspace/dexterous-arm-controllers/src/franka-arm-controllers/franka_arm/configs'

RESPONSE_TIMEOUT = 7 

CONTROL_FREQ = 60
STATE_FREQ = 200

ROTATION_VELOCITY_LIMIT = 0.5 # 1
TRANSLATION_VELOCITY_LIMIT = 1 # 2

ROTATIONAL_POSE_VELOCITY_SCALE = 1 # 2 # Scales to be used when we're using cartesian_movement
TRANSLATIONAL_POSE_VELOCITY_SCALE = 15 # 10

VELOCITY_MOVE_STEPS = 1 # For now this will be 1
POSITION_MOVE_STEPS = 30
POSITION_HOME_STEPS = 30