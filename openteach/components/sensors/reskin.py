import time
from openteach.utils.network import ZMQKeypointPublisher, ZMQKeypointSubscriber
from openteach.utils.timer import FrequencyTimer
from openteach.components import Component 

from reskin_sensor import ReSkinProcess


class ReskinSensorPublisher(Component):
    def __init__(self, stream_configs, reskin_config):
        self.reskin_publisher = ZMQKeypointPublisher(
            host=stream_configs['host'],
            port=stream_configs['port'],
        )

        self.timer = FrequencyTimer(100)
        self.reskin_config = reskin_config
        self._start_reskin()

    def _start_reskin(self):
        self.sensor_proc = ReSkinProcess(
            num_mags=self.reskin_config['num_mags'],
            port=self.reskin_config['port'],
            baudrate=100000,
            burst_mode=True,
            device_id=0,
            temp_filtered=True,
            reskin_data_struct=True
        )
        self.sensor_proc.start()
        time.sleep(0.5)
    
    def stream(self):
        self.notify_component_start('Reskin sensors')

        while True:
            try:
                self.timer.start_loop()
                reskin_state = self.sensor_proc.get_data(1)[0]
                data_dict = {}
                data_dict["timestamp"] = reskin_state.time
                data_dict["sensor_values"] = reskin_state.data
                self.reskin_publisher.pub_keypoints(data_dict, topic_name='reskin')
                self.timer.end_loop()

            except KeyboardInterrupt:
                break


class ReskinSensorSubscriber(Component):
    def __init__(self, stream_configs):
        self.reskin_subscriber = ZMQKeypointSubscriber(
            host=stream_configs['host'],
            port=stream_configs['port'],
            topic="reskin"
        )

        self.timer = FrequencyTimer(100)

    def get_sensor_state(self):
        reskin_state = self.reskin_subscriber.recv_keypoints()
        return reskin_state

    def stream(self):
        self.notify_component_start('Reskin sensors')

        while True:
            try:
                self.timer.start_loop()
                reskin_state = self.get_sensor_state()
                self.timer.end_loop()

            except KeyboardInterrupt:
                break
