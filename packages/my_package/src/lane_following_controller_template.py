#!/usr/bin/env python3

# potentially useful for question - 2.2

# import required libraries

import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
import os
import Float32

class LaneControllerNode(DTROS):
    def __init__(self, node_name):
        super(LaneControllerNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # add your code here
        
        # controller type
        self.control_type = "PID"  # it can be P or PD

        # variables
        self.image_w = 400
        
        # PID gains 
        self.proportional_gain = 0.5
        self.derivative_gain = 0.5
        self.integral_gain = 0.5
        
        # control variables
        self.prev_error = 0
        self.integral = 0
        
        # movement parameters
        self.speed = 0.5
        self.error = 0
        
        # distance tracking
        
        # initialize publisher/subscribers
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self.wheel_publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        lane_topic = f"/{vehicle_name}/lane"
        lane_sub = rospy.Subscriber("/custom_topic_name", Float32, self.yellow_lane_callback)

    

    def calculate_p_control(self):
        # add your code here
        return self.proportional_gain * self.error

    def calculate_pd_control(self):
        # add your code here
        derivative = self.error - self.prev_error 
        return self.proportional_gain * self.error + self.derivative_gain * derivative
    
    def calculate_pid_control(self):
        # add your code here
        self.integral += self.error
        derivative = self.error - self.prev_error 
        return self.proportional_gain * self.error + self.derivative_gain * derivative + self.integral_gain * self.integral

    def get_control_output(self):
        if self.control_type == "P":
            control = self.calculate_p_control()
        elif self.control_type == "PD":
            control = self.calculate_pd_control()
        elif self.control_type == "PID":
            control = self.calculate_pid_control()
        else:
            rospy.logwarn("Invalid control type!")
            control = 0.0
        self.prev_error = self.error
        return control

    def publish_cmd(self, control):
        msg = WheelsCmdStamped()
        msg.vel_left = self.base_speed - control
        msg.vel_right = self.base_speed + control
        self.wheel_publisher.publish(msg)

    def yellow_lane_callback(self, **kwargs):
        # add your code here
        pass

    # add other functions as needed

if __name__ == '__main__':
    node = LaneControllerNode(node_name='lane_controller_node')
    rospy.spin()