import rospy
import numpy as np
import time

from onrobot_rg_control.onrobot_controller import OnrobotController
from onrobot_rg_control.msg import OnRobotRGInputStamped
from onrobot_rg_control.msg import OnRobotRGOutputStamped

from copy import deepcopy as copy

ONROBOT_INPUT_TOPIC = '/OnRobotRGInputStamped'
ONROBOT_OUTPUT_TOPIC = '/OnRobotRGOutputStamped'


class DexArmControl():
    def __init__(self):
        try:
            rospy.init_node("dex_arm", disable_signals = True, anonymous = True)
        except:
            pass

    # Controller initializers
    def _init_onrobot_gripper_control(self):
        self.onrobot = OnrobotController()

        self.onrobot_joint_state = None
        rospy.Subscriber(
            ONROBOT_INPUT_TOPIC, 
            OnRobotRGInputStamped, 
            self._callback_onrobot_joint_state, 
            queue_size = 1
        )

        self.onrobot_commanded_joint_state = None
        rospy.Subscriber(
            ONROBOT_OUTPUT_TOPIC, 
            OnRobotRGOutputStamped, 
            self._callback_onrobot_commanded_joint_state, 
            queue_size = 1
        )

    # Rostopic callback functions
    def _callback_onrobot_joint_state(self, joint_state):
        self.onrobot_joint_state = joint_state

    def _callback_onrobot_commanded_joint_state(self, joint_state):
        self.onrobot_commanded_joint_state = joint_state

    # State information functions
    def get_hand_state(self):
        if self.onrobot_joint_state is None:
            return None

        raw_joint_state = copy(self.onrobot_joint_state)
        
        joint_state = dict(
            f_offset=raw_joint_state.gFOF,
            width=raw_joint_state.gGWD,
            status=raw_joint_state.gSTA,
            timestamp = raw_joint_state.header.stamp.secs + (raw_joint_state.header.stamp.nsecs * 1e-9)
        )
        return joint_state

    def get_commanded_hand_state(self):
        if self.onrobot_commanded_joint_state is None:
            return None

        raw_joint_state = copy(self.onrobot_commanded_joint_state)

        joint_state = dict(
            target_force=raw_joint_state.rGFR,
            target_width=raw_joint_state.rGWD,
            control_field=raw_joint_state.rCTR,
            timestamp = raw_joint_state.header.stamp.secs + (raw_joint_state.header.stamp.nsecs * 1e-9)     
        )
        return joint_state
        
    def get_hand_position(self):
        if self.onrobot_joint_state is None:
            return None

        return self.onrobot_joint_state.gGWD

    def get_hand_velocity(self):
        if self.onrobot_joint_state is None:
            return None

        return None

    def get_hand_torque(self):
        if self.onrobot_joint_state is None:
            return None

        return None

    def get_commanded_hand_joint_position(self):
        if self.onrobot_commanded_joint_state is None:
            return None

        return self.onrobot_commanded_joint_state.rGWD

    # Movement functions
    def move_hand(self, gripper_width):
        self.onrobot.hand_pose(gripper_width, False)

    def home_hand(self):
        self.onrobot.hand_pose(ONROBOT_HOME_VALUES)

    def reset_hand(self):
        self.home_hand()

    # Full robot commands
    def move_robot(self, gripper_width, kinova_angles):
        self.kinova.joint_movement(kinova_angles, False)
        self.onrobot.hand_pose(gripper_width, False)

    def home_robot(self):
        self.home_arm()
        self.home_hand()