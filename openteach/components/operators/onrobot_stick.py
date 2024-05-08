import zmq
import time
import numpy as np
import matplotlib.pyplot as plt 

from copy import deepcopy as copy
from .operator import Operator
from scipy.spatial.transform import Rotation
from openteach.robot.onrobot import OnrobotGripper
from openteach.utils.files import *
from openteach.utils.vectorops import *
from openteach.utils.network import ZMQKeypointSubscriber, ZMQKeypointPublisher
from openteach.utils.timer import FrequencyTimer
from openteach.constants import *
from scipy.spatial.transform import Rotation, Slerp
from scipy.spatial.transform import Rotation as R


# Rotation should be filtered when it's being sent
class Filter:
    def __init__(self, state, comp_ratio=0.6):
        self.pos_state = state[:3]
        self.ori_state = state[3:7]
        self.comp_ratio = comp_ratio

    def __call__(self, next_state):
        self.pos_state = self.pos_state[:3] * self.comp_ratio + next_state[:3] * (1 - self.comp_ratio)
        ori_interp = Slerp([0, 1], Rotation.from_quat(
            np.stack([self.ori_state, next_state[3:7]], axis=0)),)
        self.ori_state = ori_interp([1 - self.comp_ratio])[0].as_quat()
        return np.concatenate([self.pos_state, self.ori_state])


# This class is used for the kinova arm teleoperation
class OnrobotGripperStickOperator(Operator):
    def __init__(
        self,
        host, 
        controller_state_port,
    ):
        self.notify_component_start('onrobot gripper stick operator')

        # Initalizing the robot controller
        self._robot = OnrobotGripper()

        # Subscribe controller state
        self._controller_state_subscriber = ZMQKeypointSubscriber(
            host=host,
            port=controller_state_port,
            topic='controller_state'
        )
        
        time.sleep(1)
        self.gripper_width = self.robot.get_joint_state()['width']
        self.is_first_frame = True

        # Getting the bounds to perform linear transformation
        bounds_file = get_path_in_package(
            'components/operators/configs/onrobot.yaml')
        bounds_data = get_yaml_data(bounds_file)
        self.bounds = bounds_data['gripper_bounds']
        # Frequency timer
        self._timer = FrequencyTimer(VR_FREQ)

        self.start_teleop = False
        self.last_time = time.time()

    @property
    def timer(self):
        return self._timer

    @property
    def robot(self):
        return self._robot
    
    @property
    def transformed_arm_keypoint_subscriber(self):
        return self._transformed_arm_keypoint_subscriber
    
    @property
    def transformed_hand_keypoint_subscriber(self):
        return self._transformed_hand_keypoint_subscriber
    @property
    def controller_state_subscriber(self):
        return self._controller_state_subscriber
    
    def return_real(self):
        return True

    # Apply retargeted angles
    def _apply_retargeted_angles(self, log=False):
        self.controller_state = self.controller_state_subscriber.recv_keypoints()

        if self.is_first_frame:
            self.home_width = np.array(self.robot.get_joint_state()["width"])
            self.is_first_frame = False
        
        if self.controller_state.right_a:
        # Pressing A button calibrates first frame and starts teleop for right robot.
            self.start_teleop = True
            print("Starting teleop")
        
        if self.controller_state.right_b:
        # Pressing B button stops teleop. And resets calibration frames to None  for right robot.
            self.start_teleop = False
            self.home_width = np.array(self.robot.get_joint_state()["width"])
            print("Pausing teleop")

        # TODO: Add gripper control to onrobot operator

        
        if self.start_teleop:
            gripper_state = None
            if self.controller_state.right_index_trigger > 0.5:
                gripper_state = ONROBOT_CLOSE
            elif self.controller_state.right_hand_trigger > 0.5:
                gripper_state = ONROBOT_OPEN
            if gripper_state is not None:
                self.robot.move(gripper_state)
