from collections import deque
import io
import zmq
import base64
import numpy as np
import pickle
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
import logging
logger = logging.getLogger(__name__)

class VideoStreamer(object):
    def __init__(self, host, cam_port):
        self._init_socket(host, cam_port)

    def _init_socket(self, host, port):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect('tcp://{}:{}'.format(host, port))
        self.socket.setsockopt(zmq.SUBSCRIBE, b"rgb_image")

    def _get_image(self):
        raw_data = self.socket.recv()
        data = raw_data.lstrip(b"rgb_image ")
        data = pickle.loads(data)
        encoded_data = np.fromstring(base64.b64decode(data['rgb_image']), np.uint8)
        return encoded_data.tobytes()

    def yield_frames(self):
        while True:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + self._get_image() + b'\r\n')  # concat frame one by one and show result


class ReskinStreamer(object):
    def __init__(self, host, reskin_port, num_mags):
        self._init_socket(host, reskin_port)
        self.num_mags = num_mags
        self.num_sensors = int(num_mags / 5)
        self.queue_size = 1500
        self._data_history = [deque(maxlen=self.queue_size) for _ in range(self.num_sensors)]
        self._baseline = np.zeros(15*self.num_sensors)
        self._update_baseline()
        for _ in range(100):
            data = self._get_data()
            for idx in range(self.num_sensors):
                self._data_history[idx].append(data[15*idx:15*(idx+1)])

    def _init_socket(self, host, port):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.connect('tcp://{}:{}'.format(host, port))
        self.socket.setsockopt(zmq.SUBSCRIBE, bytes("reskin", 'utf-8'))

    def _get_data(self):
        raw_data = self.socket.recv()
        data = raw_data.lstrip(b"reskin ")
        data = np.array(pickle.loads(data)["sensor_values"]) - self._baseline
        # self._data_history.append(data)
        return data
    
    def _update_baseline(self):
        baseline_data = []
        for _ in range(20):
            data = self._get_data()
            baseline_data.append(data)
        self._baseline = np.median(baseline_data, axis=0)

    def yield_frames(self):
        fig = Figure()
        axs = fig.subplots(nrows=2, ncols=1)
        [ax.set_xlim(0, self.queue_size) for ax in axs]
        [ax.set_title(f"Sensor {idx+1}") for idx, ax in enumerate(axs)]
        lines = []
        for dh, ax in zip(self._data_history, axs):
            lines.append(ax.plot(dh))
        fig.tight_layout()
        update_freq = 10
        record_index = 0
        while True:
            new_data = self._get_data()
            for idx in range(self.num_sensors):
                self._data_history[idx].append(new_data[15*idx:15*(idx+1)])
            for line_set, dh, ax in zip(lines, self._data_history, axs):
                [line.set_data(range(len(dh)), d) for line,d in zip(line_set, zip(*dh))]
                ax.relim()
                ax.autoscale_view()
            # [line.set_data(range(len(self._data_history)), d) for line,d in zip(lines, zip(*self._data_history))]
            if record_index % update_freq == 0:
                output = io.BytesIO()
                FigureCanvasAgg(fig).print_png(output)
                data = np.asarray(output.getbuffer(), dtype=np.uint8)
                del output
            record_index += 1
            # encoded_data = np.fromstring(base64.b64decode(data['rgb_image']), np.uint8)
            yield (b'--frame\r\n'
                   b'Content-Type: image/png\r\n\r\n' + data.tobytes() + b'\r\n')


class MonitoringApplication(object):
    def __init__(self, configs):
        # Loading the network configurations
        self.camera_address = configs.camera_address
        self.keypoint_port = configs.keypoint_port
        self.port_offset = configs.cam_port_offset
        self.num_cams = len(configs.robot_cam_serial_numbers)

        # Initializing the streamers        
        self._init_cam_streamers()
        self._init_graph_streamer()
        logger.info("Initialized camera streamers")
        try:
            self.reskin_port = configs.reskin_publisher_port
            self.reskin_num_mags = configs.reskin_num_mags
            self._init_reskin_streamer()
            logger.info("Initialized reskin streamer")
        except AttributeError as e:
            self.reskin_num_mags = 0
            logger.warning("Reskin streamer not initialized")
            logger.info(e)
        # Initializing frequency checkers
        self._init_frequency_checkers()
        
    def _init_graph_streamer(self):
        # TODO
        pass

    def _init_frequency_checkers(self):
        # TODO - Raw keypoint frequency
        # TODO - Transformed keypoint frequency
        pass

    def _init_cam_streamers(self):
        self.cam_streamers = []
        for idx in range(self.num_cams):
            self.cam_streamers.append(
                VideoStreamer(
                    host = self.camera_address,
                    cam_port = self.port_offset + idx
                )
            )

    def _init_reskin_streamer(self):
        self.reskin_streamer = ReskinStreamer(
            host = self.camera_address,
            reskin_port = self.reskin_port,
            num_mags = self.reskin_num_mags
        )
    
    def get_reskin_streamer(self):
        return self.reskin_streamer

    def get_cam_streamer(self, id):
        return self.cam_streamers[id - 1]