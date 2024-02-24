import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from allegro_hand.controller import AllegroController
from kinova_arm.controller import KinovaController
from copy import deepcopy as copy

ALLEGRO_JOINT_STATE_TOPIC = '/allegroHand/joint_states'
ALLEGRO_COMMANDED_JOINT_STATE_TOPIC = '/allegroHand/commanded_joint_states'

KINOVA_JOINT_STATE_TOPIC = '/j2n6s300_driver/out/joint_state'
KINOVA_CARTESIAN_STATE_TOPIC = '/j2n6s300_driver/out/tool_pose'

KINOVA_HOME = [0.7261976902616409, 4.089729134350931, 1.331532741298052, 3.403334783425506, 1.4941259163823328, -2.3247042081988116]

KINOVA_CUP_SLIPPING_HOME_VALUES = [-0.45732123,  0.27744633,  0.28632292, -0.04654086, -0.56798249, 0.02050539,  0.82146782]
KINOVA_CUP_SLIPPING_HOME_VALUES_NO_STABILIZATION = [-0.47642678,  0.27087545,  0.34734036, -0.06232093, -0.67618454, 0.05119989,  0.73230404]

KINOVA_BOTTLE_CAP_OPENING_CART_HOME_VALUES = [-0.54997629,  0.29829338,  0.32790551, -0.05905117, -0.6822508 ,  0.06995045,  0.72536457]

KINOVA_BOOK_OPENNING_CART_HOME_VALUES = [-0.48370975,  0.30050284,  0.35628548, -0.0667778 , -0.69833982, 0.01928833,  0.71238345]

KINOVA_TAPE_CART_HOME_VALUES = [-0.65992874,  0.28360522,  0.27349919, -0.05751789, -0.76895034, 0.02619585,  0.63617671]

KINOVA_GAMEPAD_CART_HOME_VALUES = [-0.58856064,  0.22593541,  0.34663925, -0.05276202, -0.69327134, 0.05305005,  0.71678221]

KINOVA_BOX_HANDLE_LIFTING_CART_HOME_VALUES = [-0.42580533,  0.28117454,  0.35571384, -0.0658454, -0.6720261, -0.00949159,  0.73753321]
KINOVA_BOWL_PICKING_CART_HOME_VALUES = [-0.52456534,  0.21932657,  0.37987852,  0.04104819, -0.65619761, -0.02854434,  0.75293094]

KINOVA_PLIER_PICKING_CART_HOME_VALUES = [-0.49514708,  0.29150361,  0.34026781, -0.07374543, -0.72189188, -0.04998896,  0.68624681]
KINOVA_CARD_FLIPPING_CART_HOME_VALUES = [-0.5707984 ,  0.2903423 ,  0.31730247, -0.0505348 , -0.80014342, -0.02483405,  0.59715998] 
KINOVA_CARD_TURNING_CART_HOME_VALUES = [-0.56546915,  0.28352877,  0.32529727, -0.04745373, -0.74813676, -0.02540322,  0.66135776]
KINOVA_CUP_TURNING_CART_HOME_VALUES = [-0.5581823 ,  0.2904658 ,  0.34049535, -0.06296504, -0.72694546, -0.02468684,  0.68335658]
KINOVA_PEG_INSERTION_CART_HOME_VALUES = [-0.51666868,  0.31732166,  0.29643977, -0.10299692, -0.71015126, -0.03510073,  0.69558954]
KINOVA_MINT_OPENING_CART_HOME_VALUES = [-0.56188184,  0.31665573,  0.31719872, -0.07069087, -0.7703687, -0.03817002,  0.63251698]
KINOVA_MINT_OPENING_LIFTED_CARD_HOME_VALUES = [-0.56115389,  0.31624374,  0.31479663, -0.07334466, -0.77505636, -0.03475262,  0.62665802] # These are pretty much the same as regular MINT opening but only starts from a bit upper
KINOVA_MOUSE_SCROLLING_CART_HOME_VALUES = [-0.51619107,  0.33307174,  0.28894591, -0.07648595, -0.57457948, -0.07937531,  0.81099188]

KINOVA_HOME_VALUES = KINOVA_HOME 
KINOVA_HOME_VALUES_CART = KINOVA_CARD_FLIPPING_CART_HOME_VALUES

ALLEGRO_ORIGINAL_HOME_VALUES = [
    0, -0.17453293, 0.78539816, 0.78539816,           # Index
    0, -0.17453293,  0.78539816,  0.78539816,         # Middle
    0.08726646, -0.08726646, 0.87266463,  0.78539816, # Ring 
    1.04719755,  0.43633231,  0.26179939, 0.78539816  # Thumb
]

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

ALLEGRO_HOME_VALUES = ALLEGRO_CARD_FLIPPING_HOME_VALUES


class DexArmControl():
    def __init__(self):
        try:
            rospy.init_node("dex_arm", disable_signals = True, anonymous = True)
        except:
            pass
    
        self._init_allegro_hand_control()
        self._init_kinova_arm_control()

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

    def _init_kinova_arm_control(self):
        self.kinova = KinovaController()

        self.kinova_joint_state = None
        rospy.Subscriber(
            KINOVA_JOINT_STATE_TOPIC, 
            JointState, 
            self._callback_kinova_joint_state, 
            queue_size = 1
        )

        self.kinova_cartesian_state = None
        rospy.Subscriber(
            KINOVA_CARTESIAN_STATE_TOPIC,
            PoseStamped,
            self._callback_kinova_cartesian_state,
            queue_size = 1
        )


    # Rostopic callback functions
    def _callback_allegro_joint_state(self, joint_state):
        self.allegro_joint_state = joint_state

    def _callback_allegro_commanded_joint_state(self, joint_state):
        self.allegro_commanded_joint_state = joint_state

    def _callback_kinova_joint_state(self, joint_state):
        self.kinova_joint_state = joint_state

    def _callback_kinova_cartesian_state(self, cartesian_state):
        self.kinova_cartesian_state = cartesian_state


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

    def get_arm_cartesian_state(self):
        if self.kinova_cartesian_state is None:
            return None

        raw_cartesian_state = copy(self.kinova_cartesian_state)

        cartesian_state = dict(
            position = np.array([
                raw_cartesian_state.pose.position.x, raw_cartesian_state.pose.position.y, raw_cartesian_state.pose.position.z
            ], dtype = np.float32),
            orientation = np.array([
                raw_cartesian_state.pose.orientation.x, raw_cartesian_state.pose.orientation.y, raw_cartesian_state.pose.orientation.z, raw_cartesian_state.pose.orientation.w
            ], dtype = np.float32),
            timestamp = raw_cartesian_state.header.stamp.secs + (raw_cartesian_state.header.stamp.nsecs * 1e-9)
        )
        return cartesian_state

    def get_arm_joint_state(self):
        if self.kinova_joint_state is None:
            return None

        raw_joint_state = copy(self.kinova_joint_state)

        joint_state = dict(
            position = np.array(raw_joint_state.position[:6], dtype = np.float32),
            velocity = np.array(raw_joint_state.velocity[:6], dtype = np.float32),
            effort = np.array(raw_joint_state.effort[:6], dtype = np.float32),
            timestamp = raw_joint_state.header.stamp.secs + (raw_joint_state.header.stamp.nsecs * 1e-9)
        )
        return joint_state

    def get_arm_position(self):
        if self.kinova_joint_state is None:
            return None
        
        return np.array(self.kinova_joint_state.position, dtype = np.float32)

    def get_arm_velocity(self):
        if self.kinova_joint_state is None:
            return None
        
        return np.array(self.kinova_joint_state.velocity, dtype = np.float32)

    def get_arm_torque(self):
        if self.kinova_joint_state is None:
            return None

        return np.array(self.kinova_joint_state.effort, dtype = np.float32)

    def get_arm_cartesian_coords(self):
        if self.kinova_cartesian_state is None:
            return None

        cartesian_state  =[
            self.kinova_cartesian_state.pose.position.x,
            self.kinova_cartesian_state.pose.position.y,
            self.kinova_cartesian_state.pose.position.z,
            self.kinova_cartesian_state.pose.orientation.x,
            self.kinova_cartesian_state.pose.orientation.y,
            self.kinova_cartesian_state.pose.orientation.z,
            self.kinova_cartesian_state.pose.orientation.w
        ]
        return np.array(cartesian_state)


    # Movement functions
    def move_hand(self, allegro_angles):
        self.allegro.hand_pose(allegro_angles)

    def home_hand(self):
        self.allegro.hand_pose(ALLEGRO_HOME_VALUES)

    def reset_hand(self):
        self.home_hand()

    def move_arm(self, kinova_angles):
        self.kinova.joint_movement(kinova_angles, False)

    def move_arm_cartesian(self, kinova_cartesian_values):
        self.kinova.cartesian_movement(kinova_cartesian_values, False, True)

    def move_arm_cartesian_velocity(self, cartesian_velocity_values, duration):
        self.kinova.publish_cartesian_velocity(cartesian_velocity_values, duration)

    def home_arm(self, is_cartesian=False):
        if is_cartesian:
            self.move_arm_cartesian(KINOVA_HOME_VALUES_CART)
        else:
            self.move_arm(KINOVA_HOME_VALUES)

    def reset_arm(self):
        self.home_arm()


    # Full robot commands
    def move_robot(self, allegro_angles, kinova_angles):
        self.kinova.joint_movement(kinova_angles, False)
        self.allegro.hand_pose(allegro_angles)

    def home_robot(self, arm_cartesian):
        self.home_arm(is_cartesian=arm_cartesian) # For now we're using cartesian values
        self.home_hand()

if __name__ == '__main__':
    dex_arm = DexArmControl()
    dex_arm.home_robot(arm_cartesian=True)
