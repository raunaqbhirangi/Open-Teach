import numpy as np
import math
import sys
import os
sys.path.append(os.path.abspath('/home/kovak/LEAP_Hand_API/python'))
from openteach.utils.files import *
#from LeapController import LeapNode
from openteach.ros_links.leap_control import DexArmControl 
from openteach.robot.robot import RobotWrapper
PORT = 'ttyUSB0'
BAUDRATE = 4000000
class LeapHand(RobotWrapper):
    #Python API controller
    def __init__(self,**kwargs):
        #self.config =get_yaml_data(get_config('configs/robot/leap_xarm.yaml'))
        #self._controller = LeapNode(self.config['leap_port'],self.config['leap_baudrate'])
        self._controller = DexArmControl(robot_type='leap')
        self._data_frequency = 10
        
    @property
    def name(self):
        return 'leap'
    
    @property
    def data_frequency(self):
        return self._data_frequency
    
   
    @property
    def recorder_functions(self):
        return {
                'joint_states':self.get_joint_state,
                'commanded_joint_states':self.get_commanded_joint_state
        }
    
    def get_commanded_joint_state(self):
        return self._controller.get_commanded_joint_state()
    
    def get_joint_state(self):
        #return self._controller.get_joint_state()
        return self._controller.get_joint_state()
    
    def get_joint_position(self):
        #return self._controller.read_pos()
        return self._controller.get_hand_position()
    
    def get_joint_velocity(self):
        #return self._controller.read_vel()
        pass
    def get_joint_torque(self):
        #return self._controller.read_cur()
        pass
    def move(self, input_angles):
        #self._controller.set_leap(input_angles)
        self._controller.move_hand(input_angles)
    
    def move_allegro(self,allegro_angles):
        self._controller.move_hand_allegro(allegro_angles)
    
    def home(self):
        self._controller.home_hand()

    def move_coords(self, input_coords):
        pass

    def get_cartesian_position(self):
        pass

    