import numpy as np
import math
from openteach.robot import RobotWrapper
from LEAP_Hand_API.python.LeapController import LeapNode
from openteach.robot.robot import RobotWrapper
PORT = 'usbserial-FT7WBF78'
BAUDRATE = 3000000
class LeapHand(RobotWrapper):
    def __init__(self):
        self._controller = LeapNode(PORT,BAUDRATE)
        self._data_frequency = 30
        
    @property
    def name(self):
        return 'leap'
    
    @property
    def data_frequency(self):
        return self._data_frequency
    
   
    @property
    def recorder_functions(self):
        return {'joint_states':self._controller.get_joint_state()}
    
    def get_joint_state(self):
        return self._controller.get_joint_state()
    
    def get_joint_position(self):
        return self._controller.read_pos()

    def get_joint_velocity(self):
        return self._controller.read_vel()

    def get_joint_torque(self):
        return self._controller.read_cur()

    def move(self, input_angles):
        self._controller.set_leap(input_angles)
    
    def move_allegro(self,allegro_angles):
        self._controller.set_allegro(allegro_angles)
    
    def home(self):
        self._controller.home()

    