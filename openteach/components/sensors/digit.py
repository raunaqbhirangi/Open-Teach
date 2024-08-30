import numpy as np
from digit_interface import Digit
from openteach.components import Component
from openteach.utils.images import rotate_image, rescale_image
from openteach.utils.timer import FrequencyTimer
from openteach.utils.network import ZMQCameraPublisher, ZMQCameraSubscriber, ZMQCompressedImageTransmitter
from openteach.constants import DIGIT_FPS


class DigitSensorPublisher(Component):
    def __init__(self, stream_configs, serial_num) -> None:
        np.set_printoptions(suppress=True)
        self.rgb_publisher = ZMQCameraPublisher(
            host=stream_configs['host'],
            port=stream_configs['port']
        )

        self.timer = FrequencyTimer(DIGIT_FPS)
        # TODO: Set DIGIT FPS if required
        self.digit = Digit(serial_num)
        self.digit.connect()
    
    def stream(self):
        self.notify_component_start('digit')

        while True:
            try:
                self.timer.start_loop()
                rgb_image = self.digit.get_frame()
                # rgb_image = rotate_image(rgb_image, 180)
                # rgb_image = rescale_image(rgb_image, 0.5)
                self.rgb_publisher.pub_image(rgb_image, topic_name='digit')
                self.timer.end_loop()

            except KeyboardInterrupt:
                break


class DigitSensorSubscriber(Component):
    def __init__(self, stream_configs) -> None:
        self.rgb_subscriber = ZMQCameraSubscriber(
            host=stream_configs['host'],
            port=stream_configs['port']
        )

    def __repr__(self):
        return "reskin"

    def get_sensor_state(self):
        digit_img = self.rgb_subscriber.recv_rgb_image()
        return digit_img

    def stream(self):
        self.notify_component_start('digit')

        while True:
            try:
                self.timer.start_loop()
                rgb_image = self.rgb_subscriber.recv_rgb_image()
                self.timer.end_loop()

            except KeyboardInterrupt:
                break