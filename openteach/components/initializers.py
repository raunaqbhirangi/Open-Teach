import os
import hydra
from abc import ABC
from .recorders.image import RGBImageRecorder, DepthImageRecorder, FishEyeImageRecorder
from .recorders.robot_state import RobotInformationRecord
from .recorders.sim_state import SimInformationRecord
from .recorders.sensors import XelaSensorRecorder, ReskinSensorRecorder
from .sensors import *
from multiprocessing import Process
from openteach.constants import *
import shutil
import glob


class ProcessInstantiator(ABC):
    def __init__(self, configs):
        self.configs = configs
        self.processes = []

    def _start_component(self, configs):
        raise NotImplementedError('Function not implemented!')

    def get_processes(self):
        return self.processes


class RealsenseCameras(ProcessInstantiator):
    """
    Returns all the camera processes. Start the list of processes to start
    the camera stream.
    """
    def __init__(self, configs):
        super().__init__(configs)
        # Creating all the camera processes
        self._init_camera_processes()

    def _start_component(self, cam_idx):
        print('cam_idx: {}, stream_oculus: {}'.format(cam_idx, True if self.configs.oculus_cam == cam_idx else False))
        component = RealsenseCamera(
            stream_configs = dict(
                host = self.configs.host_address,
                port = self.configs.cam_port_offset + cam_idx
            ),
            cam_serial_num = self.configs.robot_cam_serial_numbers[cam_idx],
            cam_id = cam_idx + 1,
            cam_configs = self.configs.cam_configs,
            stream_oculus = True if self.configs.stream_oculus and self.configs.oculus_cam == cam_idx else False
        )
        component.stream()

    def _init_camera_processes(self):
        for cam_idx in range(len(self.configs.robot_cam_serial_numbers)):
            self.processes.append(Process(
                target = self._start_component,
                args = (cam_idx, )
            ))


class TeleOperator(ProcessInstantiator):
    """
    Returns all the teleoperation processes. Start the list of processes 
    to run the teleop.
    """
    def __init__(self, configs):
        super().__init__(configs)

        self._init_detector()
        self._init_keypoint_transform()
        # self._init_sensors() # Initialize the xela sensor
        self._init_visualizers()

        if configs.operate: # To run the detector only
            self._init_operator()

        if configs.debug_thumb:
            self._init_debuggers()

    def _start_component(self, configs):
        component = hydra.utils.instantiate(configs)
        component.stream()

    def _init_detector(self):
        self.processes.append(Process(
            target = self._start_component,
            args = (self.configs.robot.detector, )
        ))

    def _init_keypoint_transform(self):
        for transform_config in self.configs.robot.transforms:
            self.processes.append(Process(
                target = self._start_component,
                args = (transform_config, )
            ))

    def _init_visualizers(self):
        for visualizer_config in self.configs.robot.visualizers:
            self.processes.append(Process(
                target = self._start_component,
                args = (visualizer_config, )
            ))
        # XELA visualizer
        if self.configs.run_xela:
            for visualizer_config in self.configs.xela_visualizers:
                self.processes.append(Process(
                    target = self._start_component,
                    args = (visualizer_config, )
                ))

    def _init_operator(self):
        for operator_config in self.configs.robot.operators:
            self.processes.append(Process(
                target = self._start_component,
                args = (operator_config, )
            ))

    def _init_debuggers(self):
        for debug_config in self.configs.robot.debuggers: # NOTE: There is only one anyways
            # print(f'debug_config in initializers: {debug_config}')
            self.processes.append(Process(
                target = self._start_component,
                args = (debug_config, )
            ))


class Collector(ProcessInstantiator):
    """
    Returns all the recorder processes. Start the list of processes 
    to run the record data.
    """
    def __init__(self, configs, demo_num):
        super().__init__(configs)
        self.demo_num = demo_num
        self._storage_path = os.path.join(
            self.configs.storage_path, 
            'demonstration_{}'.format(self.demo_num)
        )
        self._create_storage_dir()

        # Initializing the recorders
        self._init_camera_recorders()
        
        if not configs.human_data:
            if configs.save_sensors:
                self._init_sensor_recorders()

            self._init_robot_recorders()
        else:
            # TODO: Initialize the keypoint saving
            self._init_keypoint_recorder()

    def _create_storage_dir(self):
        if os.path.exists(self._storage_path):
            while True:
                chk = input("Path already exists. Overwrite?")
                if chk == "y":
                    shutil.rmtree(self._storage_path)
                    os.makedirs(self._storage_path)
                    break
                elif chk == "n":
                    path_split = self._storage_path.split("_")
                    search_path = "_".join(path_split[:-1]) + "_*"
                    demos_list = glob.glob(search_path)
                    demo_id_list = [int(d.split("_")[-1]) for d in demos_list]
                    new_demo_id = max(demo_id_list) + 1
                    self._storage_path = "_".join(path_split[:-1]) + f"_{new_demo_id}"
                    print(f"Using path {self._storage_path}")
                    os.makedirs(self._storage_path)
                    break
            return 
        else:
            os.makedirs(self._storage_path)

    def _start_component(self, component):
        component.stream()

    # Obtaining the rgb and depth components
    def _start_rgb_component(self, cam_idx):
        component = RGBImageRecorder(
            host = self.configs.host_address,
            image_stream_port = self.configs.cam_port_offset + cam_idx,
            storage_path = self._storage_path,
            filename = 'cam_{}_rgb_video'.format(cam_idx)
        )
        component.stream()

    def _start_depth_component(self, cam_idx):
        component = DepthImageRecorder(
            host = self.configs.host_address,
            image_stream_port = self.configs.cam_port_offset + cam_idx + DEPTH_PORT_OFFSET,
            storage_path = self._storage_path,
            filename = 'cam_{}_depth'.format(cam_idx)
        )
        component.stream()

    def _start_keypoint_component(self, configs):
        component = hydra.utils.instantiate(
            configs,
            storage_path = self._storage_path
        )
        component.stream()

    def _init_keypoint_recorder(self):
        # First should start the keypoint detector
        self.processes.append(Process(
            target = self._start_keypoint_component,
            args = (self.configs.keypoint_detector, )
        ))

        # Start the recorder component
        self.processes.append(Process(
            target = self._start_keypoint_component,
            args = (self.configs.keypoint_recorder, )
        ))

    def _init_camera_recorders(self):
        for cam_idx in range(len(self.configs.robot_cam_serial_numbers)):
            self.processes.append(Process(
                target = self._start_rgb_component,
                args = (cam_idx, )
            ))

            self.processes.append(Process(
                target = self._start_depth_component,
                args = (cam_idx, )
            ))

    def _start_robot_component(
        self, 
        robot_configs, 
        recorder_function_key
    ):
        component = RobotInformationRecord(
            robot_configs = robot_configs,
            recorder_function_key = recorder_function_key,
            storage_path = self._storage_path
        )
        component.stream()

    def _init_robot_recorders(self):
        # Instantiating the robot classes
        for idx, robot_controller_configs in enumerate(self.configs.robot.controllers):
            if 'franka' in self.configs.robot.robot_name:
                robot_controller_configs.record = True
            for key in self.configs.robot.recorded_data[idx]:
                self.processes.append(Process(
                    target = self._start_robot_component,
                    args = (robot_controller_configs, key, )
                ))

    def _start_xela_component(self,
        controller_config
    ):
        component = XelaSensorRecorder(
            controller_configs=controller_config,
            storage_path=self._storage_path
        )
        component.stream()

    def _start_reskin_component(self,
        controller_config
    ):
        component = ReskinSensorRecorder(
            controller_configs=controller_config,
            storage_path=self._storage_path
        )
        component.stream()

    def _init_sensor_recorders(self):
        """
        For the XELA sensors and maybe microphones?
        """
        try:
            for controller_config in self.configs.xela_controllers:
                self.processes.append(Process(
                    target = self._start_xela_component,
                    args = (controller_config, )
                ))
        except AttributeError:
            pass
        
        try:
            for controller_config in self.configs.reskin_controllers:
                self.processes.append(Process(
                    target = self._start_reskin_component,
                    args = (controller_config, )
                ))
        except AttributeError:
            pass
