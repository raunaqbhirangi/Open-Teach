#import rospy
import numpy as np
import time
from copy import deepcopy as copy
from xarm import XArmAPI
from enum import Enum
import math
import random
from openteach.constants import SCALE_FACTOR, DEPLOY_FREQ, POLICY_FREQ
from scipy.spatial.transform import Rotation as R
from openteach.constants import *

class RobotControlMode(Enum):
    CARTESIAN_CONTROL = 0
    SERVO_CONTROL = 1

#Wrapper for XArm
class Robot(XArmAPI):
    def __init__(self, ip="192.168.86.230", is_radian=True, gripper_start_state=800.0):
        super(Robot, self).__init__(
            port=ip, is_radian=is_radian, is_tool_coord=False)
        self.set_gripper_enable(True)
        self.ip = ip
        self.gripper_start_state = gripper_start_state

    def clear(self):
        self.clean_error()
        self.clean_gripper_error()
        self.clean_warn()
        # self.motion_enable(enable=False)
        self.motion_enable(enable=True)

    def set_mode_and_state(self, mode: RobotControlMode, state: int = 0):
        self.set_mode(mode.value)
        self.set_state(state)
        self.set_gripper_mode(0)  # Gripper is always in position control.

    def smooth_servo_move(self,
                          target_pose: np.ndarray,
                          step_size: float = 10.0,
                          loop_rate_hz: float = 5.0,
                          tol: float = 1e-1):
        rate = 1.0 / loop_rate_hz 

        curr = np.array(self.get_position_aa()[1])
        print("Current Position:", curr)
        while True:
            step = np.zeros(6)
            diff = np.array(target_pose[:3]) - curr[:3]
            if np.all(np.abs(diff) <= tol):
                break

            step[:3] = np.clip(diff, -step_size, step_size)
            next_pose = curr + step

            print("Next:",next_pose)
            self.set_servo_cartesian_aa(next_pose,
                                        wait=True,
                                        relative=False)
            curr = next_pose
            time.sleep(rate)

        
    def reset(self):
        # Clean error
        self.clear()
        print("Reset working")
        for i in range(1):
            print(i)
            time.sleep(1)
        # self.set_mode_and_state(RobotControlMode.CARTESIAN_CONTROL, 0)
        # status = self.set_servo_angle(angle=ROBOT_HOME_JS, wait=True, is_radian=True, speed=math.radians(50))
        # for i in range(2):
        #     print(i)
        #     time.sleep(1)
        self.set_mode_and_state(RobotControlMode.SERVO_CONTROL, 0)

        modified_robot_home_pose_AA = ROBOT_HOME_POSE_AA[:]
        
        # For SpaceMouse
        # modified_robot_home_pose_AA[0] += 33
        # modified_robot_home_pose_AA[1] -= 5
        # modified_robot_home_pose_AA[2] -= 230
        
        # For Scale Demo
        # modified_robot_home_pose_AA[0] += 55
        # modified_robot_home_pose_AA[1] -= 200
        # modified_robot_home_pose_AA[2] -= 28
    
        # For Random Initialization of Cup Insertion
        # modified_robot_home_pose_AA[0] += random.uniform(-10, 10)
        # modified_robot_home_pose_AA[1] += random.uniform(-10, 10)
        
        # # For Plug Insertion
        # modified_robot_home_pose_AA[0] += 134
        # modified_robot_home_pose_AA[1] -= 105
        # modified_robot_home_pose_AA[2] -= 165
        
        # # For USB Insertion
        modified_robot_home_pose_AA[0] += 152
        modified_robot_home_pose_AA[1] -= 131
        modified_robot_home_pose_AA[2] -= 135
        
        
        # For Key Unlock
        # modified_robot_home_pose_AA[0] += 122
        # modified_robot_home_pose_AA[1] -= 131
        # modified_robot_home_pose_AA[2] -= 25
        
        # For Card Swiping
        # modified_robot_home_pose_AA[0] += 206
        # modified_robot_home_pose_AA[1] -= 175
        # modified_robot_home_pose_AA[2] -= 190
        
        # self.smooth_servo_move(modified_robot_home_pose_AA,
        #                        step_size=1,
        #                        loop_rate_hz=20.0,
        #                        tol=1)
        
        def random_outside(inner_min, inner_max, outer_min, outer_max):
            # 50/50 pick which side
            if random.random() < 0.5:
                return random.uniform(outer_min, inner_min)
            else:
                return random.uniform(inner_max, outer_max)
            
        x_offset = random_outside(-10, 10, -20, 20)
        y_offset = random_outside(-10, 10, -20, 20)

        modified_robot_home_pose_AA[0] += random.uniform(-30, 30)#x_offset#random.uniform(-30, 30)#x_offset  #random.uniform(20, 50) #random.uniform(-30, 30)
        modified_robot_home_pose_AA[1] += random.uniform(-30, 30)#y_offset#random.uniform(-30, 30)#y_offset  #random.uniform(-30, 30) #random.uniform(-30, 30)
        # modified_robot_home_pose_AA[2] += random.uniform(-30, -10)
        
        print("Set State to:", modified_robot_home_pose_AA)
        self.smooth_servo_move(modified_robot_home_pose_AA,
                               step_size=1,
                               loop_rate_hz=20.0,
                               tol=1)
        status = self.set_servo_cartesian_aa(
                    modified_robot_home_pose_AA, wait=True, relative=False)
        assert status == 0, "Failed to set robot at home joint position"
        self.set_mode_and_state(RobotControlMode.SERVO_CONTROL, 0)
        self.set_gripper_position(self.gripper_start_state, wait=True)
        # for i in range(2):
        #     print(i)
        #     time.sleep(1)
        

class DexArmControl():
    def __init__(self, ip, gripper_start_state=800.0, record_type=None):

        # if pub_port is set to None it will mean that
        # this will only be used for listening to franka and not commanding
        # try:
        #     rospy.init_node("dex_arm", disable_signals = True, anonymous = True)
        # except:
        #     pass
    
       
        #self._init_franka_arm_control(record)
        self.robot =Robot(ip, is_radian=True, gripper_start_state=gripper_start_state) 

        # self.desired_cartesian_pose = None
        # self.desired_cartesian_pose = self.get_arm_cartesian_coords()
        # self.desired_gripper_pose = 800.0
        self.idx = 0
        self.trajectory = None
        self.num_time_steps = DEPLOY_FREQ // POLICY_FREQ

    # Controller initializers
    def _init_xarm_control(self):
       
        self.robot.reset()
        
        status, home_pose = self.robot.get_position_aa()
        assert status == 0, "Failed to get robot position"
        home_affine = self.robot_pose_aa_to_affine(home_pose)
        # Initialize timestamp; used to send messages to the robot at a fixed frequency.
        last_sent_msg_ts = time.time()
   
    def get_arm_pose(self):
        status, home_pose = self.robot.get_position_aa()
        home_affine = self.robot_pose_aa_to_affine(home_pose)
        return home_affine

    def get_arm_position(self):
        joint_state =np.array(self.robot.get_servo_angle()[1])
        return joint_state

    def get_arm_velocity(self):
        raise ValueError('get_arm_velocity() is being called - Arm Velocity cannot be collected in Franka arms, this method should not be called')

    def get_arm_torque(self):
        raise ValueError('get_arm_torque() is being called - Arm Torques cannot be collected in Franka arms, this method should not be called')

    # def make_orientation_sign_consistent(self, axis):
    #     reference_axis = np.array([1, 1, 1])
    #     if np.dot(axis, reference_axis) < 0:
    #         axis = -axis
    #     return axis

    def make_orientation_sign_consistent(self, axis):
        norm_last = np.linalg.norm(self.prev_ori)
        norm_curr = np.linalg.norm(axis)
        dot_product = np.dot(axis, self.prev_ori)
        if abs(norm_last - norm_curr) < 0.2 and dot_product < 0:
            axis = -axis
        return axis


    def get_arm_cartesian_coords(self):
        status, home_pose = self.robot.get_position_aa()
        home_pose = np.array(home_pose)
        # if not hasattr(self, 'prev_ori'):
        #     self.prev_ori = home_pose[3:6]
        # else:
        #     home_pose[3:6] = self.make_orientation_sign_consistent(home_pose[3:6])
        #     self.prev_ori = home_pose[3:6]
        return home_pose

    def get_gripper_state(self):
        gripper_position=self.robot.get_gripper_position()
        gripper_pose= dict(
            position = np.array(gripper_position[1], dtype=np.float32).flatten(),
            timestamp = time.time()
        )
        return gripper_pose

    def move_arm_joint(self, joint_angles):
        self.robot.set_servo_angle(joint_angles, wait=True, is_radian=True, mvacc=80, speed=10)

    def move_arm_cartesian(self, cartesian_pos, duration=3):
        self.robot.set_servo_cartesian_aa(
                    cartesian_pos, wait=False, relative=False, mvacc=200, speed=50)
        
    def set_desired_pose(self, cartesian_pose, gripper_pose):
        self.robot.set_mode_and_state(RobotControlMode.SERVO_CONTROL, 0)
        # desired cartesian pose
        # pos
        curr_cartesian_pose = self.get_arm_cartesian_coords()
        # pos = curr_cartesian_pose[:3] + cartesian_pose[:3]
        # # ori
        # ori = curr_cartesian_pose[3:]
        # ori = R.from_rotvec(ori).as_euler('xyz')
        # sin_ori = np.sin(ori)
        # cos_ori = np.cos(ori)
        # ori = np.concatenate([sin_ori, cos_ori])
        # ori = ori + cartesian_pose[3:]
        # sin_ori, cos_ori = ori[:3], ori[3:]
        # ori = np.arctan2(sin_ori, cos_ori)
        # # convert to axis angle
        # ori = R.from_euler('xyz', ori).as_rotvec()      
        # # desired
        # desired_cartesian_pose = np.concatenate([pos, ori])
        # find current matrix
        pos_curr = curr_cartesian_pose[:3]
        ori_curr = curr_cartesian_pose[3:]
        r_curr = R.from_rotvec(ori_curr).as_matrix()
        matrix_curr = np.eye(4)
        matrix_curr[:3, :3] = r_curr
        matrix_curr[:3, 3] = pos_curr
        # find transformation matrix
        pos_delta = cartesian_pose[:3]
        ori_delta = cartesian_pose[3:]
        r_delta = R.from_rotvec(ori_delta).as_matrix()
        matrix_delta = np.eye(4)
        matrix_delta[:3, :3] = r_delta
        matrix_delta[:3, 3] = pos_delta
        # find desired matrix
        matrix_desired = matrix_curr @ matrix_delta
        # matrix_desired = matrix_delta @ matrix_curr
        # pos_desired = matrix_desired[:3, 3]
        pos_desired = pos_curr + pos_delta
        r_desired = matrix_desired[:3, :3]
        ori_desired = R.from_matrix(r_desired).as_rotvec()
        desired_cartesian_pose = np.concatenate([pos_desired, ori_desired])
        # print("current_cartesian_pose", curr_cartesian_pose)
        # print("desired_cartesian_pose", desired_cartesian_pose)
        # desired_cartesian_pose = curr_cartesian_pose + cartesian_pose

        # desired gripper pose
        self.apply_gripper=False
        ############### variant 1
        # if not hasattr(self, 'desired_gripper_pose'):
        #     self.desired_gripper_pose = gripper_pose
        #     self.gripper_change_count = 0
        #     self.apply_gripper=True
        # elif gripper_pose != self.desired_gripper_pose:
        #         if self.gripper_change_count >=3:
        #             self.desired_gripper_pose = gripper_pose
        #             self.gripper_change_count = 0
        #             self.apply_gripper=True
        #         else:
        #             self.gripper_change_count += 1
        ############### variant 2
        # if not hasattr(self, 'desired_gripper_pose') or gripper_pose != self.desired_gripper_pose:
        #     self.desired_gripper_pose = gripper_pose
        #     self.apply_gripper=True
        ############### variant 3
        if not hasattr(self, 'desired_gripper_pose'):
            self.desired_gripper_pose = min(1, max(0, gripper_pose)) * 800
            self.apply_gripper=True
        elif self.desired_gripper_pose > 400 and gripper_pose < 0.6: #0.5:
            self.desired_gripper_pose = 0
            self.apply_gripper=True
        elif self.desired_gripper_pose < 400 and gripper_pose > 0.7:
            self.desired_gripper_pose = 800
            self.apply_gripper=True

        # Get minjerk trajectory
        self.trajectory = self.min_jerk_trajectory_generator(curr_cartesian_pose, desired_cartesian_pose, self.num_time_steps)
        self.idx = 0

    def min_jerk_trajectory_generator(self, current, target, num_steps):
        # Generate a minimum jerk trajectory between current and target
        # Reference: https://en.wikipedia.org/wiki/Minimum_jerk_trajectory
        # The trajectory is generated in cartesian and gripper space
        trajectory = []
        for time in range(1, num_steps+1):
            t = time / num_steps
            trajectory.append(current + (target - current) * (10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5))
        return trajectory

    def arm_control(self, cartesian_pose):
        # while self.robot.has_error:
        #     self.robot.clear()
        #     self.robot.set_mode_and_state(RobotControlMode.SERVO_CONTROL, 0)
        self.move_arm_cartesian(cartesian_pose)
    
    def continue_control(self):
        while self.robot.has_error:
            self.robot.clear()
            self.robot.set_mode_and_state(RobotControlMode.SERVO_CONTROL, 0)
        
        if self.trajectory is None or self.idx >= self.num_time_steps:
            return

        self.arm_control(self.trajectory[self.idx])
        # print(self.idx, self.num_time_steps)
        # if self.idx >= 0.5 * self.num_time_steps:
        # print("Setting gripper status", self.desired_gripper_pose)
        if self.apply_gripper and self.idx == self.num_time_steps - 1:
            self.set_gripper_status(self.desired_gripper_pose)
            self.apply_gripper = False
        self.idx += 1
        
    def get_arm_joint_state(self):
        joint_positions =np.array(self.robot.get_servo_angle()[1])
        joint_state = dict(
            position = np.array(joint_positions, dtype=np.float32),
            timestamp = time.time()
        )
        return joint_state
        
    def get_cartesian_state(self):
        # status,current_pos=self.robot.get_position_aa()
        current_pos = self.get_arm_cartesian_coords()

        pos, ori = current_pos[:3], current_pos[3:]
        # ori = R.from_rotvec(ori).as_euler('xyz')
        cartesian_state = dict(
            position = np.array(pos, dtype=np.float32).flatten(),
            orientation = np.array(ori, dtype=np.float32).flatten(),
            timestamp = time.time()
        ) 

        # cartesian_state = dict(
        #     position = np.array(current_pos[0:3], dtype=np.float32).flatten(),
        #     orientation = np.array(current_pos[3:], dtype=np.float32).flatten(),
        #     timestamp = time.time()
        # )

        return cartesian_state

    def home_arm(self):
        self.move_arm_cartesian(BIMANUAL_RIGHT_HOME, duration=5)

    def reset_arm(self):
        self.home_arm()

    # Full robot commands
    def move_robot(self,arm_angles):
        self.robot.set_servo_angle(angle=arm_angles,is_radian=True)
        
    def home_robot(self):
        self.home_arm()

    def set_gripper_status(self, position):
        self.robot.set_gripper_position(position, wait=True)

    def robot_pose_aa_to_affine(self,pose_aa: np.ndarray) -> np.ndarray:
        """Converts a robot pose in axis-angle format to an affine matrix.
        Args:
            pose_aa (list): [x, y, z, ax, ay, az] where (x, y, z) is the position and (ax, ay, az) is the axis-angle rotation.
            x, y, z are in mm and ax, ay, az are in radians.
        Returns:
            np.ndarray: 4x4 affine matrix [[R, t],[0, 1]]
        """

        rotation = R.from_rotvec(pose_aa[3:]).as_matrix()
        translation = np.array(pose_aa[:3]) / SCALE_FACTOR

        return np.block([[rotation, translation[:, np.newaxis]],
                        [0, 0, 0, 1]])