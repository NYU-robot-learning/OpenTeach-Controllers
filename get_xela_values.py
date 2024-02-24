import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import rospy
import pickle
import sys
import signal
from copy import deepcopy as copy

from xela_server.srv import XelaSensorXYZ
from xela_server.msg import xServerMsg
from xela_sensors.utils import get_tactile_index, get_curved_tactile_index


XELA_SERVER_TOPIC = '/xServTopic'
XELA_NUM_SENSORS = 15 # 3 in thumb 4 in other 3 fingers 
XELA_NUM_TAXELS = 16 # Number of taxels in one tactile sensor 
XELA_NUM_PALM_TAXELS = 24
XELA_NUM_FINGERTIP_TAXELS = 30 
XELA_NUM_FINGER_TAXELS = 16
XELA_NUM_PALM_SENSORS = 3
XELA_NUM_FINGERTIP_SENSORS = 4
XELA_NUM_FINGER_SENSORS = 11


class XelaSensorControl():
    def __init__(self):
        try:
            rospy.init_node('xela_sensor', disable_signals=True, anonymous=True)
        except:
            pass 
        self._init_xela_sensor_control()

    # Controller initializer
    # Only initilizes the listeners - since we can't control the xela sensors
    def _init_xela_sensor_control(self):
        self.xela_sensor_state = None 
        rospy.Subscriber(
            XELA_SERVER_TOPIC,
            xServerMsg,
            self._callback_xela_sensors_state,
            queue_size = 1
        )        

    # Rostopic callback function
    def _callback_xela_sensors_state(self, xela_state):
        self.xela_sensor_state = xela_state 

    # State information function - more constructed information from the xela readings
    def get_sensor_state(self):
        if self.xela_sensor_state is None: 
            return None
        
        raw_xela_state = copy(self.xela_sensor_state)
        curr_sensor_values = np.zeros((XELA_NUM_SENSORS, XELA_NUM_TAXELS, 3)) 
        for taxel_id in range(XELA_NUM_SENSORS * XELA_NUM_TAXELS):
            sensor_id, tactile_id = get_tactile_index(taxel_id)
            raw_taxel_state = raw_xela_state.points[taxel_id]
            curr_sensor_values[sensor_id, tactile_id, :] = [
            raw_taxel_state.point.x,
            raw_taxel_state.point.y,
            raw_taxel_state.point.z
            ]
        timestamp = raw_xela_state.header.stamp.secs + (raw_xela_state.header.stamp.nsecs * 1e-9)
        return curr_sensor_values, timestamp
    
class XelaCurvedSensorControl():
    def __init__(self):
        try:
            rospy.init_node('xela_sensor', disable_signals=True, anonymous=True)
            print("ROS node created")
        except:
            pass 

        self._init_xela_sensor_control()
        self.cnt=0
        self.cnt_fingertip=0
        self.cnt_finger=0

    # Controller initializer
    # Only initilizes the listeners - since we can't control the xela sensors
    def _init_xela_sensor_control(self):
        self.xela_sensor_state = None 
        rospy.Subscriber(
            XELA_SERVER_TOPIC,
            xServerMsg,
            self._callback_xela_sensors_state,
            queue_size = 1
        )       

    # Rostopic callback function
    def _callback_xela_sensors_state(self, xela_state):
        self.xela_sensor_state = xela_state 

    # State information function - more constructed information from the xela readings
    def get_sensor_state(self):
        if self.xela_sensor_state is None: 
            return None
        
        raw_xela_state = copy(self.xela_sensor_state)
        curr_sensor_palm_values = np.zeros((XELA_NUM_PALM_SENSORS, XELA_NUM_PALM_TAXELS, 3)) 
        curr_sensor_fingertip_values = np.zeros((XELA_NUM_FINGERTIP_SENSORS, XELA_NUM_FINGERTIP_TAXELS, 3))
        curr_sensor_finger_values = np.zeros((XELA_NUM_FINGER_SENSORS, XELA_NUM_FINGER_TAXELS, 3))

        for taxel_id in range(368):
            sensor_id, tactile_id = get_curved_tactile_index(taxel_id)
            raw_taxel_state = raw_xela_state.points[taxel_id]
            
            if sensor_id>14:
                curr_sensor_palm_values[sensor_id-15, tactile_id, :] = [
                    raw_taxel_state.point.x,
                    raw_taxel_state.point.y,
                    raw_taxel_state.point.z
                ]
            elif sensor_id==0 or sensor_id==3 or sensor_id==7 or sensor_id==11:

                curr_sensor_fingertip_values[int(sensor_id/3), tactile_id, :] = [
                    raw_taxel_state.point.x,
                    raw_taxel_state.point.y,
                    raw_taxel_state.point.z
                ]
            else:

                if sensor_id > 11:
                    sensor_id -= 4
                elif sensor_id > 7:
                    sensor_id -= 3
                elif sensor_id > 3:
                    sensor_id -= 2
                else:
                    sensor_id -= 1
                
                curr_sensor_finger_values[sensor_id, tactile_id, :] = [
                    raw_taxel_state.point.x,
                    raw_taxel_state.point.y,
                    raw_taxel_state.point.z
                ]

        timestamp = raw_xela_state.header.stamp.secs + (raw_xela_state.header.stamp.nsecs * 1e-9)
        return curr_sensor_palm_values,curr_sensor_fingertip_values, curr_sensor_finger_values, timestamp

if __name__ == '__main__':
    xela_controller = XelaCurvedSensorControl()
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        sensor_state = xela_controller.get_sensor_state()
        if not sensor_state is None:
            curr_sensor_palm_values, curr_sensor_fingertip_values,curr_sensor_finger_values, timestamp = sensor_state
            if curr_sensor_palm_values is not None and curr_sensor_fingertip_values is not None and curr_sensor_finger_values is not None:
                print('curr_palm_sensor_values[0]: {} - {}\n-------'.format(curr_sensor_palm_values.shape, np.mean(curr_sensor_palm_values)))
                # print('curr_palm_values: {}'.format(curr_sensor_palm_values))
                print('curr_sensor_fingertip_values[0]: {} - {}\n-------'.format(curr_sensor_fingertip_values.shape, np.mean(curr_sensor_fingertip_values)))
                print('curr_sensor_finger_values[0]: {} - {}\n-------'.format(curr_sensor_finger_values.shape, np.mean(curr_sensor_finger_values)))

        rate.sleep()