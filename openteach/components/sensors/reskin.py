from openteach.utils.timer import FrequencyTimer
from openteach.components import Component 
from openteach.ros_links.reskin_sensor_stream import ReskinSensorStream


class ReskinSensors(Component):
    def __init__(self):
        self._controller = ReskinSensorStream()
        self.timer = FrequencyTimer(100)

    def __repr__(self) -> str:
        return "reskin"

    def get_sensor_state(self):
        reskin_state = self._controller.get_sensor_state()
        if reskin_state is not None:
            normalized_state = dict(
                sensor_values=reskin_state['sensor_values'],
                timestamp=reskin_state['timestamp']
            )
            return normalized_state
        return reskin_state  # NOTE: This can be None as well

    def stream(self):
        # # Starting the xela stream
        self.notify_component_start('Reskin sensors')
        print(f"Started the Reskin sensor pipeline for the hand")

        while True: 
            try:
                self.timer.start_loop()
                reskin_state = self.get_sensor_state()
                self.timer.end_loop()

            except KeyboardInterrupt:
                break
