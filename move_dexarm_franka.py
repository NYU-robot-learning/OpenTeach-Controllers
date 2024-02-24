import rospy
import numpy as np
import time

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from allegro_hand.controller import AllegroController
from franka_arm.controller import FrankaController
from copy import deepcopy as copy

from deoxys.utils import transform_utils

from franka_arm.constants import *
from franka_arm.utils import generate_cartesian_space_min_jerk

ALLEGRO_JOINT_STATE_TOPIC = '/allegroHand/joint_states'
ALLEGRO_COMMANDED_JOINT_STATE_TOPIC = '/allegroHand/commanded_joint_states'

FRANKA_HOME = [-1.5208185 ,  1.5375434 ,  1.4714179 , -1.8101345 ,  0.01227421, 1.8809032 ,  0.67484516]

FRANKA_PINCH_GRASP_HOME=[ 0.51763135, -0.11029206,  0.20979846, -0.29405248, -0.6412179 ,-0.6262204 ,  0.3319947 ]

FRANKA_FINGER_GAIT_HOME=[ 0.5032102 , -0.15280019,  0.28621227, -0.66453695,  0.2848382 ,0.29123807,  0.62644905]

FRANKA_GENERALIZATION_HOME=[ 0.58253205, -0.16104414,  0.22045994, -0.29267502, -0.6780309 ,
       -0.601653  ,  0.30435067]

FRANKA_PINCH_GRASP_HOME_2=[ 0.50860757, -0.12697428,  0.21223924, -0.25529376, -0.6617287 ,
       -0.6327886 ,  0.31067488]

FRANKA_PINCH_GRASP_HOME_3=[ 0.5095174 , -0.12722133,  0.21704282, -0.2528554 , -0.6639972 ,
       -0.6329741 ,  0.3074334 ]

FRANKA_BOX_OPENING_HOME=[ 0.46098545, -0.16476414,  0.36661303, -0.24184185, -0.68061227,
       -0.62618107,  0.29355928]


FRANKA_CARD_SLIDING_HOME=[ 0.52667034, -0.14614083,  0.30523318, -0.30356798, -0.63736796,
       -0.65045786,  0.28020203]

FRANKA_SPRAY_FINAL_HOME=[ 0.63151896, -0.15155165,  0.34743842, -0.65991306, -0.29673183,
       -0.28334734,  0.62942755]


STATE_RECO = [0.41281763, -0.18762423,  0.31808898, -0.244461  , -0.6497351 ,
       -0.6484832 ,  0.31233397]

TOUCHING_JOYSTICK_HOME = [ 0.60063416,  0.11174913,  0.32961157, -0.27150202, -0.9355028 ,
       -0.22226536,  0.04146236]

FRANKA_UNSTACK_PEG_INSERT_HOME = [ 0.67565846, -0.20553339,  0.3595517 , -0.2807263 , -0.68395925,
       -0.61089325,  0.28319964]

FRANKA_CHROLOX_BOX_HOME=[ 0.53635776, -0.24796237,  0.4312788 , -0.26402378, -0.70186293,
       -0.59157765,  0.29616854]
# FRANKA_HOME_CART = FRANKA_UNSTACK_PEG_INSERT_HOME

FRANKA_HOME_RIGHT = [ 0.72288483, -0.11261073,  0.2998383 , -0.6884363 ,  0.28586346,
        0.22752155,  0.6265553 ]
FRANKA_HOME_LEFT = [ 0.40616864, -0.13264222,  0.34513497, -0.69290453,  0.24063879, 0.24200177,  0.63514656]

FRANKA_HOME_TOP = [ 0.5329852 , -0.12784682,  0.5017979 , -0.62965477,  0.28064543,
        0.24625002,  0.68127376]
FRANKA_HOME_BOTTOM = [ 0.5518923 , -0.11076009,  0.26439014, -0.65690774,  0.28998524,
        0.2800887 ,  0.63712716]

FRANKA_HOME_FRONT = [ 0.58340204,  0.00512866,  0.28497437, -0.67081755,  0.2729959 ,
        0.2922382 ,  0.6245591 ]
FRANKA_HOME_BACK = [ 0.57853353, -0.27230647,  0.30454764, -0.6524815 ,  0.30215853,
        0.25030887,  0.648316  ]

FRANKA_SPONGE_FLIPPING_HOME = [ 0.51642907, -0.16153438,  0.255893  , -0.25410226, -0.7360989 ,
       -0.5786347 ,  0.2424304 ]

FRANKA_TENNIS_BALL_BALANCE = [ 0.3708882 , -0.15503047,  0.3374355 , -0.7693669 ,  0.31496233,
        0.17315511,  0.5281007 ]

FRANKA_HOME_CART = FRANKA_SPONGE_FLIPPING_HOME
FRANKA_HOME_SPRAYING=[ 0.61587566, -0.13917102,  0.34310198, -0.6934492 ,  0.2856983 ,
        0.24294531,  0.6152091 ]
FRANKA_HOME_CARD_PICKING=[0.61267734, -0.13478118,  0.29464763, -0.2734049 , -0.62293154,
       -0.67993194,  0.273676]

# FRANKA_JOYSTICK_HOME= [0.6542806, -0.1803534, 0.20465541, -0.65155935, -0.285941, -0.33317026, 0.6186321]
FRANKA_JOYSTICK_HOME=[0.54843676, -0.21178848, 0.23713006, -0.5746991, -0.4362173, -0.28005797, 0.6332479]

FRANKA_SPONGE_PICKING_HOME =[0.5849516 , -0.21625192,  0.31164503, -0.29108885, -0.64113265,
       -0.6541724 ,  0.276178]

FRANKA_CARD_ROTATE_HOME= [ 0.6322507 , -0.20697606,  0.38260865, -0.27492493, -0.68193525,
       -0.61302567,  0.28910235]

FRANKA_FIDGET_SPIN_HOME=[ 0.572959  , -0.20279118,  0.3522907 , -0.2664079 , -0.68519473,
       -0.6107567 ,  0.29412773 ]

FRANKA_FIDGET_SPIN_HOME_2= [0.58109486, -0.19741082,  0.34838045, -0.6434198 ,  0.27150613,
        0.28201678,  0.65784645]

FRANKA_MUSIC_BOX_OPEN_HOME=[ 0.61939764, -0.21312736,  0.21798871, -0.30505708, -0.6352805 ,
       -0.640904  ,  0.30430415]
# KINOVA_HOME_VALUES = KINOVA_HOME
# KINOVA_HOME_VALUES_CART = KINOVA_CARD_FLIPPING_CART_HOME_VALUES

ALLEGRO_ORIGINAL_HOME_VALUES = [
    0, -0.17453293, 0.78539816, 0.78539816,           # Index
    0, -0.17453293,  0.78539816,  0.78539816,         # Middle
    0.08726646, -0.08726646, 0.87266463,  0.78539816, # Ring
    1.04719755,  0.43633231,  0.26179939, 0.78539816  # Thumb
]

ALLEGRO_FIDGET_SPIN_HOME_VALUES = [-0.01963584, -0.3025789 ,  0.7155776 ,  0.77011245,  0.14686283,
        0.28431472,  1.4802241 ,  0.55495733,  0.08159674, -0.30089182,
        0.8113944 ,  0.7665314 ,  1.5427239 , -0.01945792,  0.25269178,
        0.967461  ]

ALLEGRO_FIDGET_SPIN_HOME_VALUES_2=[-0.01385863,  0.04565038,  0.17872623,  0.11324087, -0.08313352,
        0.28285718,  1.4099818 ,  0.6718367 ,  0.01832195,  0.2681494 ,
        0.51885575,  0.25052962,  0.89152855,  0.73832875,  1.0988388 ,
        0.77801365]
ALLEGRO_CUP_SLIPPING_HOME_VALUES = [ # This is used for BOWL PICKING as well
    -0.0658244726801581, 0.11152991296986751, 0.036465840916854717, 0.29693057660614736, # Index
    -0.09053422635521813, 0.21657171862672447, -0.17754325611897262, 0.27011271061536507, # Middle
    0.012094523852233988, 0.11196786731996372, -0.017784060790178313, 0.2670852707825862, # Ring
    0.8499175389966154, 0.3062633015641964, 0.7989875369900138, 0.46722180902731736 # Thumb
]

ALLEGRO_BOTTLE_CAP_OPENING_HOME_VALUES = [ # This is used for BOOK_OPENING AS WELL - I think this could be used for BOX HANDLE LIFTING as well
    -0.10406068960941592, 0.19625765888591587, 0.5455416026548555, 0.6413563160772814, # Index
    -0.13881295376850777, 0.13003522437082457, 0.6107575764680238, 0.42471420968995055, # Middle
    -0.19278830009506917, 0.1009857230372155, 0.5310970321033471, 0.446055473538483, # Ring
    0.6771463953596024, 0.3493525152060263, 1.1241743903532158, 0.5125013678262919
]

ALLEGRO_GAMEPAD_HOME_VALUES = [
	-0.16619228801773384, 0.32481266483797294, 0.8601691699986072, 0.5649435451227687,
    -0.13332195291895735, 0.15834478938130678, 1.1840795787354972, 0.39027226748215954,
    -0.19282213441914253, 0.08267445601587486, 1.199441197337786, 0.4190408662740487,
    0.7506308611120873, 0.5571486316166259, 1.1012251916691909, 0.4804005486841599
]

ALLEGRO_CARD_FLIPPING_HOME_VALUES = [
    -0.055535682780422854, 0.03907992617559122, 0.5147753186418194, 0.748602214657691,
    -0.10264223897187673, 0.011392517338036723, 0.3647570384028202, 0.773809080963551,
    -0.011080934919122539, 0.06960798594413742, 0.3502775197909466, 0.7669559795594911,
    1.1622167757299338, 0.405469152239451, 0.12007841939021036, 0.6319962825242011
]

ALLEGRO_BOWL_UNSTACKING_HOME_VALUES = ALLEGRO_CUP_SLIPPING_HOME_VALUES
ALLEGRO_PLIER_PICKING_HOME_VALUES = ALLEGRO_ORIGINAL_HOME_VALUES
ALLEGRO_CARD_TURNING_HOME_VALUES = ALLEGRO_ORIGINAL_HOME_VALUES
ALLEGRO_CUP_TURNING_HOME_VALUES = ALLEGRO_ORIGINAL_HOME_VALUES
ALLEGRO_PEG_INSERTION_HOME_VALUES = ALLEGRO_ORIGINAL_HOME_VALUES
ALLEGRO_MINT_OPENING_HOME_VALUES = [
    0.007651593318878476, 0.27805874641647516, 0.49409139803226626, 0.12588787585164785,
    -0.11940214584593481, 0.19662629377894955, 0.5199197897356908, 0.06314802003100513,
    0.009800427120003513, 0.12339007191971564, 0.48730888519866017, 0.1442982664591681,
    1.2516348347765962, 0.1655223256916746, 0.6311178279922637, 0.022541493408132687
]
ALLEGRO_MOUSE_SCROLLING_HOME_VALUES = [
    0.011837843617687385, 0.3705835548277471, 0.4435625821909271, 0.10201898715399182,
    -0.06633480223730141, 0.3588573620378023, 0.41581310499283447, 0.03867149647724294,
    0.0003212507904771605, 0.21369718456951864, 0.514610971649544, 0.1742635087813526,
    0.5957417758094358, 0.3454505224384596, 0.9916339259151139, 0.031421117547491176
]

ALLEGRO_FINGER_POINTING_FORWARD = [
    0.14163120921862815, 0.06365357849641412, 0.05082193379357495, 0.07581252958041149,
    -0.04011047588646893, 1.585805141594104, 1.37351389640721, 0.5598133638775273,
    0.07060749686764364, 1.620531612882041, 1.4151827827014114, 0.39609849082458176,
    1.0705467351068876, 0.1784475981493598, 0.7917874205637172, 0.7046755438451018
]

ALLEGRO_TENNIS_BALL_BALANCE = [
    -0.12001511917492877, 1.503717395429282, 1.5439861148917513, 0.9384742972785305,
    0.07533026768786673, 1.5531833267663846, 1.4159111833007707, 0.9252956324905043,
    0.21236999537721069, 1.5942046999319133, 1.2605492449645248, 1.2662389983786881,
    0.6169414680683349, 0.28764760323566185, 0.6508623366527991, 1.2294979699380868
    ]

ALLEGRO_HOME_VALUES = ALLEGRO_ORIGINAL_HOME_VALUES
FRANKA_TEA_PICKING=[0.53916454, -0.23157556,  0.25026545, -0.1642998 , -0.48592967,
       -0.8089102 ,  0.28730178]


class DexArmControl():
    def __init__(self, record=False):

        # if pub_port is set to None it will mean that
        # this will only be used for listening to franka and not commanding
        try:
            rospy.init_node("dex_arm", disable_signals = True, anonymous = True)
        except:
            pass

        self._init_allegro_hand_control()
        self._init_franka_arm_control(record)

    # Controller initializers
    def _init_allegro_hand_control(self):
        self.allegro = AllegroController()

        self.allegro_joint_state = None
        rospy.Subscriber(
            ALLEGRO_JOINT_STATE_TOPIC,
            JointState,
            self._callback_allegro_joint_state,
            queue_size = 1
        )

        self.allegro_commanded_joint_state = None
        rospy.Subscriber(
            ALLEGRO_COMMANDED_JOINT_STATE_TOPIC,
            JointState,
            self._callback_allegro_commanded_joint_state,
            queue_size = 1
        )

    def _init_franka_arm_control(self, record=False):

        if record:
            print('RECORDING IN FRANKA!')

        self.franka = FrankaController(record)

    # Rostopic callback functions
    def _callback_allegro_joint_state(self, joint_state):
        self.allegro_joint_state = joint_state

    def _callback_allegro_commanded_joint_state(self, joint_state):
        self.allegro_commanded_joint_state = joint_state

    # State information functions
    def get_hand_state(self):
        if self.allegro_joint_state is None:
            return None

        raw_joint_state = copy(self.allegro_joint_state)

        joint_state = dict(
            position = np.array(raw_joint_state.position, dtype = np.float32),
            velocity = np.array(raw_joint_state.velocity, dtype = np.float32),
            effort = np.array(raw_joint_state.effort, dtype = np.float32),
            timestamp = raw_joint_state.header.stamp.secs + (raw_joint_state.header.stamp.nsecs * 1e-9)
        )
        return joint_state

    def get_commanded_hand_state(self):
        if self.allegro_commanded_joint_state is None:
            return None

        raw_joint_state = copy(self.allegro_commanded_joint_state)

        joint_state = dict(
            position = np.array(raw_joint_state.position, dtype = np.float32),
            velocity = np.array(raw_joint_state.velocity, dtype = np.float32),
            effort = np.array(raw_joint_state.effort, dtype = np.float32),
            timestamp = raw_joint_state.header.stamp.secs + (raw_joint_state.header.stamp.nsecs * 1e-9)
        )
        return joint_state

    def get_hand_position(self):
        if self.allegro_joint_state is None:
            return None

        return np.array(self.allegro_joint_state.position, dtype = np.float32)

    def get_hand_velocity(self):
        if self.allegro_joint_state is None:
            return None

        return np.array(self.allegro_joint_state.velocity, dtype = np.float32)

    def get_hand_torque(self):
        if self.allegro_joint_state is None:
            return None

        return np.array(self.allegro_joint_state.effort, dtype = np.float32)

    def get_commanded_hand_joint_position(self):
        if self.allegro_commanded_joint_state is None:
            return None

        return np.array(self.allegro_commanded_joint_state.position, dtype = np.float32)

    def get_arm_osc_position(self):
        current_pos, current_axis_angle = copy(self.franka.get_osc_position())
        current_pos = np.array(current_pos, dtype=np.float32).flatten()
        current_axis_angle = np.array(current_axis_angle, dtype=np.float32).flatten()

        osc_position = np.concatenate(
            [current_pos, current_axis_angle],
            axis=0
        )

        return osc_position

    def get_arm_cartesian_state(self):
        current_pos, current_quat = copy(self.franka.get_cartesian_position())

        cartesian_state = dict(
            position = np.array(current_pos, dtype=np.float32).flatten(),
            orientation = np.array(current_quat, dtype=np.float32).flatten(),
            timestamp = time.time()
        )

        return cartesian_state


    def get_arm_joint_state(self):
        joint_positions = copy(self.franka.get_joint_position())
        # print('joint_position: {}'.format(joint_positions))

        joint_state = dict(
            position = np.array(joint_positions, dtype=np.float32),
            timestamp = time.time()
        )

        return joint_state

    def get_arm_pose(self):
        pose = copy(self.franka.get_pose())

        pose_state = dict(
            position = np.array(pose, dtype=np.float32),
            timestamp = time.time()
        )

        return pose_state

    def get_arm_position(self):

        joint_state = self.get_arm_joint_state()
        return joint_state['position']


    def get_arm_velocity(self):
        raise ValueError('get_arm_velocity() is being called - Arm Velocity cannot be collected in Franka arms, this method should not be called')

    def get_arm_torque(self):
        raise ValueError('get_arm_torque() is being called - Arm Torques cannot be collected in Franka arms, this method should not be called')


    def get_arm_cartesian_coords(self):
        current_pos, current_quat = copy(self.franka.get_cartesian_position())

        current_pos = np.array(current_pos, dtype=np.float32).flatten()
        current_quat = np.array(current_quat, dtype=np.float32).flatten()

        cartesian_coord = np.concatenate(
            [current_pos, current_quat],
            axis=0
        )

        return cartesian_coord

    # Movement functions
    def move_hand(self, allegro_angles):
        self.allegro.hand_pose(allegro_angles)

    def home_hand(self):
        self.allegro.hand_pose(ALLEGRO_HOME_VALUES)

    def reset_hand(self):
        self.home_hand()

    def move_arm_joint(self, joint_angles):
        self.franka.joint_movement(joint_angles)

    def move_arm_cartesian(self, cartesian_pos, duration=3):
        # Moving
        start_pose = self.get_arm_cartesian_coords()
        poses = generate_cartesian_space_min_jerk(
            start = start_pose,
            goal = cartesian_pos,
            time_to_go = duration,
            hz = self.franka.control_freq
        )

        for pose in poses:
            self.arm_control(pose)

        # Debugging the pose difference
        last_pose = self.get_arm_cartesian_coords()
        pose_error = cartesian_pos - last_pose
        debug_quat_diff = transform_utils.quat_multiply(last_pose[3:], transform_utils.quat_inverse(cartesian_pos[3:]))
        angle_diff = 180*np.linalg.norm(transform_utils.quat2axisangle(debug_quat_diff))/np.pi
        print('Absolute Pose Error: {}, Angle Difference: {}'.format(
            np.abs(pose_error[:3]), angle_diff
        ))

    def arm_control(self, cartesian_pose):
        self.franka.cartesian_control(cartesian_pose=cartesian_pose)

    def home_arm(self):
        self.move_arm_cartesian(FRANKA_JOYSTICK_HOME, duration=5)


    def reset_arm(self):
        self.home_arm()

    # Full robot commands
    def move_robot(self, allegro_angles, arm_angles):
        self.franka.joint_movement(arm_angles, False)
        self.allegro.hand_pose(allegro_angles)

    def home_robot(self):
        self.home_hand()
        self.home_arm() # For now we're using cartesian values


if __name__ == '__main__':
    dex_arm = DexArmControl()
    dex_arm.home_robot()
    # dex_arm.move_arm_cartesian(FRANKA_HOME_RIGHT)
    # dex_arm.move_arm_cartesian(FRANKA_HOME_LEFT, duration=5)

    # dex_arm.home_robot(arm_cartesian=True)
    # dex_arm.move_arm_cartesian(FRANKA_HOME_RIGHT)

    # VR_FREQ = 20
    # DURATION = 2
    # # Homing
    # start_pose = dex_arm.get_arm_cartesian_coords()
    # final_pose = FRANKA_HOME_CART
    # home_poses = generate_cartesian_space_min_jerk(
    #     start = start_pose,
    #     goal = final_pose,
    #     time_to_go = DURATION,
    #     hz = VR_FREQ
    # )
    # # breakpoint()
    # for i in range(VR_FREQ*DURATION):
    #     dex_arm.move_arm_cartesian(home_poses[i], num_steps=1)


    # # Go right
    # start_pose = dex_arm.get_arm_cartesian_coords()
    # final_pose = FRANKA_HOME_RIGHT
    # poses = generate_cartesian_space_min_jerk(
    #     start = start_pose,
    #     goal = final_pose,
    #     time_to_go = DURATION,
    #     hz = VR_FREQ
    # )
    # for i in range(VR_FREQ*DURATION):
    #     dex_arm.move_arm_cartesian(poses[i], num_steps=1)


    # start_pose = dex_arm.get_arm_cartesian_coords()
    # final_pose = FRANKA_HOME_LEFT
    # poses = generate_cartesian_space_min_jerk(
    #     start = start_pose,
    #     goal = final_pose,
    #     time_to_go = DURATION,
    #     hz = VR_FREQ
    # )

    # i = 0
    # # rate = rospy.Rate(VR_FREQ)
    # curr_time = time.time()
    # # print('start_time: {}'.format())
    # for i in range(VR_FREQ*DURATION):
    #     dex_arm.move_arm_cartesian(poses[i], num_steps=1)
    # end_time = time.time()
    # print('duration: {}'.format(end_time - curr_time))

    # dex_arm.move_arm_cartesian_velocity([0,0,0,0,0,1],0.5)

    # before_move = dex_arm.get_arm_cartesian_coords()
    # dex_arm.move_arm_cartesian_velocity([-1,0,0,0,0,0],2)
    # after_move = dex_arm.get_arm_cartesian_coords()
    # print(f'first_movement: {after_move - before_move}')
    # dex_arm.move_arm_cartesian_velocity([0,0,-0.1,0,0,0],0.5)
    # dex_arm.move_arm_cartesian_velocity([1,0,0,0,0,0],0.5)
    # dex_arm.move_arm_cartesian_velocity([0,0,0.1,0,0,0],0.5)




    # before_move = dex_arm.get_arm_cartesian_coords()

    # for curr_dir in range(4):

    #     for i in range(5):

    #         if curr_dir == 0:
    #             averaged_velocity = np.array([-1, 0,0,0,0,0]) # Left
    #         elif curr_dir == 1:
    #             averaged_velocity = np.array([0,0,-1, 0,0,0]) # Down
    #             # averaged_velocity = np.array([-0.1, 0,0,0,0,0]) # Left
    #         elif curr_dir == 2:
    #             averaged_velocity = np.array([1, 0,0,0,0,0]) # Right
    #             # averaged_velocity = np.array([0.1, 0,0,0,0,0]) # Right
    #         elif curr_dir == 3:
    #             averaged_velocity = np.array([0,0,1, 0,0,0]) # Up
    #             # averaged_velocity = np.array([-0.1, 0,0,0,0,0]) # Left

    #         dex_arm.move_arm_cartesian_velocity(np.array([-1, 0,0,0,0,0])*50, duration=1/VR_FREQ) # , 1/VR_FREQ

    # after_move = dex_arm.get_arm_cartesian_coords()
    # print(f'second_movement: {after_move - before_move}')

    # np.save('positions_without_hand.npy', positions)

    # self.direction_counter += 1

    # dex_arm.move_arm_cartesian_velocity([-1,0,0,0,0,0],0.5)
    # dex_arm.move_arm_cartesian_velocity([0,0,-0.2,0,0,0],0.5)
    # dex_arm.move_arm_cartesian_velocity([1,0,0,0,0,0],0.5)
    # dex_arm.move_arm_cartesian_velocity([0,0,0.2,0,0,0],0.5)
    # dex_arm.move_arm_cartesian_velocity([0,-0.5,0,0,0,0],0.5)
    # dex_arm.move_arm_cartesian_velocity([0,0,0.2,0,0,0],0.5)
    # dex_arm.move_arm_cartesian(FRANKA_HOME_BOTTOM)
    # dex_arm.move_arm_cartesian(FRANKA_HOME_TOP)

    # dex_arm.get_arm_cartesian_coords()
    # dex_arm.move_arm_cartesian_velocity([0,0,0,0,0,-1],1)
    # dex_arm.get_arm_cartesian_coords()

    # dex_arm.move_arm_cartesian(FRANKA_HOME_BOTTOM)
    # dex_arm.move_arm_cartesian(FRANKA_HOME_TOP)
    # dex_arm.move_arm_cartesian(FRANKA_HOME_LEFT)
    # dex_arm.move_arm_cartesian(FRANKA_HOME_RIGHT)
    # dex_arm.move_arm_cartesian(FRANKA_HOME_BACK)
    # dex_arm.move_arm_cartesian(FRANKA_HOME_FRONT)

    # Real hand rotation
    # z: on/geri sag negatif, sol pozitif - etrafinda
    # y: - sag/sol - sag pozitif - sol negativ
    # x: - asagi/yukari - yukari pozitif - asagi negatif

    # Gercek robotta translation
    # x: sag/sol - sol pozitif - sag negatif
    # z: top/bottom - top pozitif - bottom negatif
    # y: back/front - front negatif - back pozitif
