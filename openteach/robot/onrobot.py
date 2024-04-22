import numpy as np

from .robot import RobotWrapper
from openteach.ros_links.onrobot_control import DexArmControl # Modified from https://github.com/NYU-robot-learning/DIME-Controllers


class OnrobotGripper(RobotWrapper):
    def __init__(self, record_type=None) -> None:
        super().__init__()
        self._controller = DexArmControl()

        self._data_frequency = 300

    @property
    def name(self):
        return 'onrobot'

    # TODO: Check these functions and replace appropriately
    @property
    def recorder_functions(self):
        return {
            'joint_states': self.get_joint_state, 
            'commanded_joint_states': self.get_commanded_joint_state
        }

    @property
    def data_frequency(self):
        return self._data_frequency

    # State information functions
    def get_joint_state(self):
        return self._controller.get_hand_state()

    def get_commanded_joint_state(self):
        return self._controller.get_commanded_hand_state()

    def get_joint_position(self):
        pass

    def get_joint_velocity(self):
        pass

    def get_joint_torque(self):
        pass

    def get_cartesian_state(self):
        pass


    # def get_joint_position(self):
    #     return self._controller.get_hand_position()

    # def get_joint_velocity(self):
    #     return self._controller.get_hand_velocity()

    def get_joint_torque(self):
        return self._controller.get_hand_torque()

    def get_commanded_joint_position(self):
        return self._controller.get_commanded_hand_joint_position()

    def set_random_position(self):
        # TODO: Fix limits
        self.move(np.random.uniform(0,1))

    # Movement functions
    def home(self):
        self._controller.home_hand()

    def move_coords(self, input_coords):
        self._controller.move_hand(input_coords)

    def move(self, angles):
        print("EXEC on onrobot", angles)
        self._controller.move_hand(angles)