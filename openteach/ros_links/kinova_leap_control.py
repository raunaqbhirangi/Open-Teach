import rospy
import numpy as np
import time
from LEAP_Hand_API.ros_module.LeapController import LeapController
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from kinova_arm.controller import KinovaController
from copy import deepcopy as copy
import math
KINOVA_JOINT_STATE_TOPIC = '/j2n6s300_driver/out/joint_state'
KINOVA_CARTESIAN_STATE_TOPIC = '/j2n6s300_driver/out/tool_pose'
KINOVA_HOME_VALUES = np.array([4.22, 3.84, 1.47, 6.04, 0.90, 0.60])
LEAP_JOINT_STATE_TOPIC = "/leaphandnode/joint_states"
LEAP_COMMANDED_JOINT_STATE_TOPIC = "/leaphandnode/commanded_joint_states"
LEAP_COMMANDED_JOINT_STATE_TOPIC_ALLEGRO = "/leaphand_node/cmd_allegro"
LEAP_HOME_VALUES= np.array([math.pi,math.pi,math.pi,math.pi,math.pi,math.pi,math.pi,math.pi,math.pi,math.pi,math.pi,math.pi,math.pi,math.pi,math.pi])
class DexArmControl():
    def __init__(self,record_type=None, robot_type='kinova'):
        try:
            rospy.init_node("dex_arm", disable_signals = True, anonymous = True)
        except:
            pass

        if robot_type == 'both':
            self._init_leap_hand_control()
            self._init_kinova_arm_control()
        elif robot_type == 'leap':
            self._init_leap_hand_control()
        elif robot_type == 'kinova':
            self._init_kinova_arm_control()

        

    # Controller initializers
    def _init_leap_hand_control(self):
        self.leap = LeapController()

        self.leap_joint_state = None
        rospy.Subscriber(
            LEAP_JOINT_STATE_TOPIC, 
            JointState, 
            self._callback_leap_joint_state, 
            queue_size = 1
        )

        self.leap_commanded_joint_state = None
        rospy.Subscriber(
            LEAP_COMMANDED_JOINT_STATE_TOPIC, 
            JointState, 
            self._callback_leap_commanded_joint_state, 
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

        print("Entering Kinova Arm Control")

    # Rostopic callback functions
    def _callback_leap_joint_state(self, joint_state):
        self.leap_joint_state = joint_state

    def _callback_leap_commanded_joint_state(self, joint_state):
        self.leap_commanded_joint_state = joint_state

    def _callback_kinova_joint_state(self, joint_state):
        self.kinova_joint_state = joint_state

    def _callback_kinova_cartesian_state(self, cartesian_state):
        self.kinova_cartesian_state = cartesian_state


    # State information functions
    def get_hand_state(self):
        if self.leap_joint_state is None:
            return None

        raw_joint_state = copy(self.leap_joint_state)

        joint_state = dict(
            position = np.array(raw_joint_state.position, dtype = np.float32),
            velocity = np.array(raw_joint_state.velocity, dtype = np.float32),
            effort = np.array(raw_joint_state.effort, dtype = np.float32),
            timestamp = raw_joint_state.header.stamp.secs + (raw_joint_state.header.stamp.nsecs * 1e-9)
        )
        return joint_state

    def get_commanded_hand_state(self):
        if self.leap_commanded_joint_state is None:
            return None

        raw_joint_state = copy(self.leap_commanded_joint_state)

        joint_state = dict(
            position = np.array(raw_joint_state.position, dtype = np.float32),
            velocity = np.array(raw_joint_state.velocity, dtype = np.float32),
            effort = np.array(raw_joint_state.effort, dtype = np.float32),
            timestamp = raw_joint_state.header.stamp.secs + (raw_joint_state.header.stamp.nsecs * 1e-9)
        )
        return joint_state
        
    def get_hand_position(self):
        if self.leap_joint_state is None:
            return None

        return np.array(self.leap_joint_state.position, dtype = np.float32)

    def get_hand_velocity(self):
        if self.leap_joint_state is None:
            return None

        return np.array(self.leap_joint_state.velocity, dtype = np.float32)

    def get_hand_torque(self):
        if self.leap_joint_state is None:
            return None

        return np.array(self.leap_joint_state.effort, dtype = np.float32)

    def get_commanded_hand_joint_position(self):
        if self.leap_commanded_joint_state is None:
            return None

        return np.array(self.leap_commanded_joint_state.position, dtype = np.float32)

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
    def move_hand(self, leap_angles):
        self.leap.hand_pose(leap_angles)
    
    def move_hand_allegro(self,allegro_angles):
        self.leap.hand_pose_allegro(allegro_angles)

    def home_hand(self):
        self.leap.hand_pose(LEAP_HOME_VALUES)

    def reset_hand(self):
        self.home_hand()

    def move_arm(self, kinova_angles):
        self.kinova.joint_movement(kinova_angles, False)

    def move_arm_cartesian(self, kinova_cartesian_values):
        self.kinova.cartesian_movement(kinova_cartesian_values, False, is_quaternion=True)

    def move_arm_cartesian_velocity(self, cartesian_velocity_values, duration):
        self.kinova.publish_cartesian_velocity(cartesian_velocity_values, duration)

    def home_arm(self):
        self.kinova.joint_movement(KINOVA_HOME_VALUES, False)

    def reset_arm(self):
        self.home_arm()


    # Full robot commands
    def move_robot(self, leap_angles, kinova_angles):
        self.kinova.joint_movement(kinova_angles, False)
        self.leap.hand_pose(leap_angles)

    def home_robot(self):
        self.home_arm()
        self.home_hand()