#!/usr/bin/env python
import rospy 
import message_filters
import time 
import json
from geometry_msgs.msg import Twist
from turtlesim.msg import Color
from turtlesim.msg import Pose
from package_tutorial_rupp.msg import Aggregate
from package_tutorial_rupp.srv import Set_Log_Time
from rospy_message_converter import json_message_converter

# This one works!
class simple_log:
    def __init__(self):
        self.path = rospy.get_param(param_name="path_to_package", default="./")
        self.sleep_time = rospy.get_param(param_name="log_sleep_time", default=5.0)
        self.message = Aggregate()
        self.state_sub = rospy.Subscriber("rupp/state", Aggregate, self.save_state)
        self.s = rospy.Service('rupp/set_log_time', Set_Log_Time, self.handle_sleep_time)
    
    def save_state(self, state_msg):
        self.message = state_msg
        self.log_to_file()
        time.sleep(self.sleep_time)

    def handle_sleep_time(self, srv_msg):
        self.sleep_time = srv_msg.new_time
        return "Time set succesfully!"

    def log_to_file(self):
        msg = json_message_converter.convert_ros_message_to_json(self.message)
        with open(self.path + '/bot_state.log', 'a', newline='\n') as file:
            # file.write('\n----------\n' + str(msg) + '\n===========\n')
            file.write(msg + '\n')

        rospy.loginfo("Appended to bot_state.log")


def main():
    node_name = rospy.get_param(param_name="log_node_name", default="log_node")
    rospy.init_node(node_name, anonymous=True)
    agg = simple_log()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__ == '__main__':
    main()