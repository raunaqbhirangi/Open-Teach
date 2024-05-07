import numpy as np
import rospy

from copy import deepcopy as copy
from reskin.msg import ReskinStamped

RESKIN_SERVER_TOPIC = '/ReskinStamped'

class ReskinSensorStream():
    def __init__(self):
        try:
            rospy.init_node('reskin_sensor', disable_signals=True, anonymous=True)
        except:
            pass 

        self.num_mags = rospy.get_param("/reskin/num_mags")
        self._init_reskin_stream()

    # Controller initializer
    # Only initilizes the listeners - since we can't control the xela sensors
    def _init_reskin_stream(self):
        self.reskin_current_state = None
        self.reskin_sensor_state = None 
        # self.curr_sensor_values = np.zeros((XELA_NUM_SENSORS, XELA_NUM_TAXELS, 3))
        rospy.Subscriber(
            RESKIN_SERVER_TOPIC,
            ReskinStamped,
            self._callback_reskin_sensor_state,
            queue_size = 1
        )        

    # Rostopic callback function
    def _callback_reskin_sensor_state(self, reskin_state):
        self.reskin_sensor_state = reskin_state 

    # State information function - more constructed information from the xela readings
    def get_sensor_state(self):
        if self.reskin_sensor_state is None: 
            return None

        raw_reskin_state = copy(self.reskin_sensor_state)
        curr_sensor_values = raw_reskin_state.data

        reskin_state = dict(
            sensor_values = curr_sensor_values,
            timestamp = raw_reskin_state.header.stamp.secs + (raw_reskin_state.header.stamp.nsecs * 1e-9)
        )
        return reskin_state


if __name__ == '__main__':
    reskin_stream = ReskinSensorStream()
    # rospy.init_node('xela_sensor')
    # TODO: Fix streaming rate
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        reskin_state = reskin_stream.get_sensor_state()
        if reskin_state is not None:
            curr_sensor_values = reskin_state['sensor_values']
            print('curr_sensor_values: {}\n-------'.format(curr_sensor_values.shape))
        rate.sleep()


