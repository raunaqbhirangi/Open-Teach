import rospy
import numpy as np
import time
import sys
import os
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from copy import deepcopy as copy
sys.path.append(os.path.abspath('/home/kovaak/LEAP_Hand_API'))
from ros_module.LeapController import LeapController

LEAP_COMMANDED_JOINT_STATE_TOPIC = '/leaphand_node/cmd_leap'
LEAP_JOINT_STATE_TOPIC = '/leaphand_node/joint_states'

LEAP_HOME_VALUES = np.full((16, 1), np.pi)


class DexArmControl():
    def __init__(self, record_type=None, robot_type='leap'):

        # if pub_port is set to None it will mean that
        # this will only be used for listening to franka and not commanding
        try:
            rospy.init_node("dex_arm", disable_signals = True, anonymous = True)
        except:
            pass
    
        if robot_type == 'leap':
            self._init_leap_hand_control()
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

   

    # Rostopic callback functions
    def _callback_leap_joint_state(self, joint_state):
        self.leap_joint_state = joint_state

    def _callback_leap_commanded_joint_state(self, joint_state):
        self.leap_commanded_joint_state = joint_state

    # State information functions
    def get_hand_state(self):
        if self.leap_joint_state is None:
            return None

        raw_joint_state = copy(self.leap_joint_state)

        joint_state = dict(
            position = np.array(raw_joint_state.position, dtype = np.float32),
            #velocity = np.array(raw_joint_state.velocity, dtype = np.float32),
            #effort = np.array(raw_joint_state.effort, dtype = np.float32),
            #timestamp = raw_joint_state.header.stamp.secs + (raw_joint_state.header.stamp.nsecs * 1e-9)
        )
        return joint_state

    def get_commanded_hand_state(self):
        if self.leap_commanded_joint_state is None:
            return None

        raw_joint_state = copy(self.leap_commanded_joint_state)

        joint_state = dict(
            position = np.array(raw_joint_state.position, dtype = np.float32),
            #velocity = np.array(raw_joint_state.velocity, dtype = np.float32),
            #effort = np.array(raw_joint_state.effort, dtype = np.float32),
            #timestamp = raw_joint_state.header.stamp.secs + (raw_joint_state.header.stamp.nsecs * 1e-9)
        )
        return joint_state
        
    def get_hand_position(self):
        if self.leap_joint_state is None:
            return None

        return np.array(self.leap_joint_state.position, dtype = np.float32)
    '''
    def get_hand_velocity(self):
        if self.leap_joint_state is None:
            return None

        return np.array(self.leap_joint_state.velocity, dtype = np.float32)

    def get_hand_torque(self):
        if self.leap_joint_state is None:
            return None

        return np.array(self.leap_joint_state.effort, dtype = np.float32)
'''
    def get_commanded_hand_joint_position(self):
        if self.leap_commanded_joint_state is None:
            return None

        return np.array(self.leap_commanded_joint_state.position, dtype = np.float32)

    
    # Movement functions
    def move_hand(self, leap_angles):
        self.leap.hand_pose(leap_angles)

    def move_hand_allegro(self, allegro_angles):
        self.leap.hand_pose_allegro(allegro_angles)
    def home_hand(self):
        self.leap.hand_pose(LEAP_HOME_VALUES)

    def reset_hand(self):
        self.home_hand()

    
    # Full robot commands
    def move_robot(self, leap_angles, arm_angles):
        self.leap.hand_pose(leap_angles)

    def home_robot(self):
        self.home_hand()
        # For now we're using cartesian values
