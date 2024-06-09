import numpy as np
import math
from allegro.allegro_kdl import AllegroKDL
from openteach.robot import RobotWrapper
from openteach.ros_links.kinova_leap_control import DexArmControl
from openteach.utils.files import get_yaml_data, get_path_in_package
from openteach.robot.robot import RobotWrapper

class LeapHand(RobotWrapper):
    def __init__(self,**kwargs):
        self._controller = DexArmControl(robot_type = 'leap')
        self.data_frequency = 300
        self._kdl_solver = AllegroKDL()
        self._joint_limit_config = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_link_info.yaml"))['links_info']
        #self.leapNode = LeadNode()
        

    @property
    def name(self):
        return 'leap'
    @property
    def data_frequency(self):
        return self.data_frequency
    
    @property
    def recorder_functions(self):
        return {
            'joint_states': self.get_joint_state, 
            'commanded_joint_states': self.get_commanded_joint_state
        }
    
    def get_joint_state(self):
        return self._controller.get_hand_state()
    
    def get_commanded_joint_state(self):
        return self._controller.get_commanded_hand_state()
    
    def get_commanded_joint_position(self):
        return self._controller.get_commanded_hand_joint_position()
    
    def get_joint_position(self):
        #return self._controller.read_pos()
        return self._controller.get_hand_position()

    def get_joint_velocity(self):
        #return self._controller.read_vel()
        return self._controller.get_hand_velocity()
    
    def get_joint_torque(self):
        return self._controller.get_hand_torque()
    
    def get_commanded_joint_position(self):
        return self._controller.get_commanded_hand_joint_position()
    
    
    
    # Kinematics functions

    def get_fingertip_coords(self, joint_positions):
        return self._kdl_solver.get_fingertip_coords(joint_positions)

    def _get_joint_state_from_coord(self, index_tip_coord, middle_tip_coord, ring_tip_coord, thumb_tip_coord):
        return self._kdl_solver.get_joint_state_from_coord(
            index_tip_coord, 
            middle_tip_coord, 
            ring_tip_coord, 
            thumb_tip_coord,
            seed = self.get_joint_position()
        )


    # Movement functions
    def home(self):
        self._controller.home_hand()


    def move_coords(self, fingertip_coords):
        desired_angles = self._get_joint_state_from_coord(
            index_tip_coord = fingertip_coords[0:3],
            middle_tip_coord = fingertip_coords[3:6],
            ring_tip_coord = fingertip_coords[6:9],
            thumb_tip_coord = fingertip_coords[9:]
        )

        self._controller.move_hand(desired_angles)

    def move(self, angles):
        self._controller.move_hand(angles)