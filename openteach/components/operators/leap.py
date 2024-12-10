from copy import deepcopy as copy
from openteach.utils.network import ZMQKeypointSubscriber, ZMQKeypointPublisher
from .operator import Operator
import math

from shapely.geometry import Point, Polygon 
from shapely.ops import nearest_points
from .calibrators.leap import OculusThumbBoundCalibrator
from openteach.robot.Leap.leap import LeapHand
from openteach.robot.Leap.leap_retargeters import LeapKDLControl, LeapJointControl
from openteach.utils.files import *
from openteach.utils.vectorops import *
from openteach.utils.timer import FrequencyTimer
from openteach.constants import *

class LeapHandOperator(Operator):
    def __init__(self,host,transformed_keypoints_port, finger_configs):
        self.notify_component_start('leap hand operator')
        #joint_publisher_port = None
        self._host, self._port = host, transformed_keypoints_port
        # Subscriber for the transformed hand keypoints
        self._transformed_hand_keypoint_subscriber = ZMQKeypointSubscriber(
            host = self._host.strip(),
            port = self._port,
            topic = 'transformed_hand_coords'
        )
        # Subscriber for the transformed arm frame
        self._transformed_arm_keypoint_subscriber = ZMQKeypointSubscriber(
            host = self._host.strip(),
            port = self._port,
            topic = 'transformed_hand_frame'
        )
        '''
        #Pulibsher for the commanded hand pose
        self._commanded_hand_keypoint_publisher = ZMQKeypointPublisher(
            host = self.host.strip(),
            port = joint_publisher_port
        )

        # Publisher for the actual hand pose
        self._actual_hand_keypoint_publisher = ZMQKeypointPublisher(
            host = self.host.strip(),
            port = joint_publisher_port
        )
        '''
        # Initializing the  finger configs
        self.finger_configs = finger_configs
        
        #Initializing the solvers for leap hand
        self.fingertip_solver = LeapKDLControl()
        self.finger_joint_solver = LeapJointControl()

        # Initializing the robot controller
        self._robot = LeapHand()

        self.moving_average_queues = {
            'thumb': [],
            'index': [],
            'middle': [],
            'ring': []
        }

        self.joint_angle_queues = []

        # Calibrating to get the thumb bounds
        self._calibrate_bounds()

        # Getting the bounds for the leap hand
        leap_bounds_path = get_path_in_package('components/operators/configs/leap.yaml')
        self.leap_bounds = get_yaml_data(leap_bounds_path)
        self.bound_info = get_yaml_data(get_path_in_package("robot/Leap/configs/leap_bounds.yaml"))
        self.rotatory_scaling_factors = self.bound_info['rotatory_scaling_factors']
        self._timer = FrequencyTimer(VR_FREQ)

        # Using 3 dimensional thumb motion or two dimensional thumb motion
        if self.finger_configs['three_dim']:
            self.thumb_angle_calculator = self._get_3d_thumb_angles
        else:
            self.thumb_angle_calculator = self._get_2d_thumb_angles
        self.finger_angle_calculator = self.fingertip_solver.finger_3D_motion
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
    def actual_hand_publisher(self):
        return self._actual_hand_keypoint_publisher

    @property
    def commanded_hand_publisher(self):
        return self._commanded_hand_keypoint_publisher
    
    # This function differentiates between the real robot and simulation
    def return_real(self):
        return True

    # Calibrate the thumb bounds
    def _calibrate_bounds(self):
        self.notify_component_start('calibration')
        calibrator = OculusThumbBoundCalibrator(self._host, self._port)
        self.hand_thumb_bounds,self.hand_finger_bounds = calibrator.get_bounds() # Provides thumb : [top_right,bottom_right,ring_bottom,ring_top]
        #finger{index,middle,ring: 'top right','top left', 'bottom left' , 'bottom right', 'lowest z', 'highest z'}
        print(f'THUMB BOUNDS IN THE OPERATOR: {self.hand_thumb_bounds}')

    def _get_finger_coords(self):
        raw_keypoints = self.transformed_hand_keypoint_subscriber.recv_keypoints()
        return dict(
            index = np.vstack([raw_keypoints[0], raw_keypoints[OCULUS_JOINTS['index']]]),
            middle = np.vstack([raw_keypoints[0], raw_keypoints[OCULUS_JOINTS['middle']]]),
            ring = np.vstack([raw_keypoints[0], raw_keypoints[OCULUS_JOINTS['ring']]]),
            thumb =  np.vstack([raw_keypoints[0], raw_keypoints[OCULUS_JOINTS['thumb']]])
        )
    
     # Get robot thumb angles when moving only in 2D motion
    def _get_2d_thumb_angles(self, thumb_keypoints, curr_angles):
        for idx, thumb_bounds in enumerate(self.hand_thumb_bounds):
            if coord_in_bound(thumb_bounds[:4], thumb_keypoints[:2]) > -1:
                return self.fingertip_solver.thumb_motion_2D(
                    hand_coordinates = thumb_keypoints, 
                    xy_hand_bounds = thumb_bounds[:4],
                    yz_robot_bounds = self.leap_bounds['thumb_bounds'][idx]['projective_bounds'],
                    robot_x_val = self.leap_bounds['x_coord'],
                    moving_avg_arr = self.moving_average_queues['thumb'], 
                    curr_angles = curr_angles
                )
        
        return curr_angles

    # Get robot thumb angles when moving in 3D motion
    def _get_3d_thumb_angles(self, thumb_keypoints, curr_angles):
        # We will be using polygon implementations of shapely library to test this
        planar_point = Point(thumb_keypoints[-1])
        planar_thumb_bounds = Polygon(self.hand_thumb_bounds[:4])
        # Get the closest point from the thumb to the point within the bounds
        closest_point = nearest_points(planar_thumb_bounds, planar_point)[0]
        closest_point_coords = [closest_point.x, closest_point.y, thumb_keypoints[-1][2]]
        last_joint= calculate_angle(
                thumb_keypoints[-3][:2],
                thumb_keypoints[-2][:2],
                thumb_keypoints[-1][:2]
            )* self.rotatory_scaling_factors["thumb"]
        
        return self.fingertip_solver.thumb_motion_3D(
            hand_coordinates = closest_point_coords,
            xy_hand_bounds = self.hand_thumb_bounds[:4],
            yz_robot_bounds = self.leap_bounds['thumb_bounds'][0]['projective_bounds'], # NOTE: We assume there is only one bound now
            z_hand_bound = self.hand_thumb_bounds[4],
            x_robot_bound = self.leap_bounds['thumb_bounds'][0]['x_bounds'],
            moving_avg_arr = self.moving_average_queues['thumb'], 
            curr_angles = curr_angles,
            hand_joint_angle = last_joint
        )
    

    #Generate frozen angles for the fingers
    def _generate_frozen_angles(self, joint_angles, finger_type):
        for idx in range(ALLEGRO_JOINTS_PER_FINGER):
            if idx > 0:
                joint_angles[idx + ALLEGRO_JOINT_OFFSETS[finger_type]] = 0.05
            else:
                joint_angles[idx + ALLEGRO_JOINT_OFFSETS[finger_type]] = 0

        return joint_angles
    
    # Apply the retargeted angles to the robot
    def _apply_retargeted_angles(self):
        hand_keypoints = self._get_finger_coords()
        #actual_leap_hand_angle = self.robot.get_joint_position()
        #self.actual_hand_publisher.pub_keypoints(actual_leap_hand_angle,"actual hand position")
        desired_joint_angles = copy(self.robot.get_joint_position())
        # Movement for the index finger with option to freeze the finger
        if not self.finger_configs['freeze_index'] and not self.finger_configs['no_index']:
            desired_joint_angles = self.finger_angle_calculator(#self.finger_joint_solver.calculate_finger_angles
                finger_type = 'index',
                finger_keypoints = hand_keypoints['index'],#hand_keypoints['index]
                hand_bound = self.hand_finger_bounds['index'],
                robot_bound = self.leap_bounds['index'],
                curr_angles = desired_joint_angles,
                moving_avg_arr = self.moving_average_queues['index']
            )
            #print((desired_joint_angles[0]-math.pi)*180/math.pi)
        elif self.finger_configs['freeze_index']:
            self._generate_frozen_angles(desired_joint_angles, 'index')
        else:
            print("No index")
            pass
        
        # Movement for the index finger with option to freeze the finger
        if not self.finger_configs['freeze_middle'] and not self.finger_configs['no_middle']:
            desired_joint_angles = self.finger_angle_calculator(#self.finger_joint_solver.calculate_finger_angles
                finger_type = 'middle',
                finger_keypoints = hand_keypoints['middle'],#hand_keypoints['index]
                hand_bound = self.hand_finger_bounds['middle'],
                robot_bound = self.leap_bounds['middle'],
                curr_angles = desired_joint_angles,
                moving_avg_arr = self.moving_average_queues['middle']
            )
            #print((desired_joint_angles[0]-math.pi)*180/math.pi)
        elif self.finger_configs['freeze_middle']:
            self._generate_frozen_angles(desired_joint_angles, 'middle')
        else:
            print("No middle")
            pass

        '''
        # Movement for the middle finger option to freeze the finger joint angle based. 
        if not self.finger_configs['freeze_middle'] and not self.finger_configs['no_middle']:
            desired_joint_angles = self.finger_joint_solver.calculate_finger_angles(
                finger_type = 'middle',
                finger_joint_coords = hand_keypoints['middle'],
                curr_angles = desired_joint_angles,
                moving_avg_arr = self.moving_average_queues['middle']
            )
        elif self.finger_configs['freeze_middle']:
            self._generate_frozen_angles(desired_joint_angles, 'middle')
        else :
            print("No Middle")
            pass
            
        '''

        # Movement for the index finger with option to freeze the finger
        if not self.finger_configs['freeze_ring'] and not self.finger_configs['no_ring']:
            desired_joint_angles = self.finger_angle_calculator(#self.finger_joint_solver.calculate_finger_angles
                finger_type = 'ring',
                finger_keypoints = hand_keypoints['ring'],#hand_keypoints['index]
                hand_bound = self.hand_finger_bounds['ring'],
                robot_bound = self.leap_bounds['ring'],
                curr_angles = desired_joint_angles,
                moving_avg_arr = self.moving_average_queues['ring']
            )
            #print((desired_joint_angles[0]-math.pi)*180/math.pi)
        elif self.finger_configs['freeze_ring']:
            self._generate_frozen_angles(desired_joint_angles, 'ring')
        else:
            print("No ring")
            pass
        '''
        # Movement for the ring finger option to freeze the finger
        if not self.finger_configs['freeze_ring'] and not self.finger_configs['no_ring']:
            desired_joint_angles = self.finger_joint_solver.calculate_finger_angles(
                finger_type = 'ring',
                finger_joint_coords = hand_keypoints['ring'],
                curr_angles = desired_joint_angles,
                moving_avg_arr = self.moving_average_queues['ring']
            )
        elif self.finger_configs['freeze_ring']:
            self._generate_frozen_angles(desired_joint_angles, 'ring')
        else: 
            print("No ring")
            pass
        '''
        
        # Movement for the thumb finger with option to freeze the finger
        if not self.finger_configs['freeze_thumb'] and not self.finger_configs['no_thumb']:
            desired_joint_angles = self.thumb_angle_calculator(hand_keypoints['thumb'], desired_joint_angles) # Passing just the tip coordinates
        elif self.finger_configs['freeze_thumb']:
            self._generate_frozen_angles(desired_joint_angles, 'thumb')
        else:
            print("No thumb")
            pass
        #print(self.moving_average_queues)
        # Move the robot 
        #self.commanded_hand_publisher.pub_keypoints(desired_joint_angles,"commanded_hand_joint")
        #print(desired_joint_angles)
        desired_joint_angles = moving_average(np.array(desired_joint_angles),self.joint_angle_queues,self.bound_info['time_steps_joint'])
        self.robot.move(list(desired_joint_angles))