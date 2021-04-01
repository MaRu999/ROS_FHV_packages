#!/usr/bin/env python
import rospy 
import message_filters
from geometry_msgs.msg import Twist
from turtlesim.msg import Color
from turtlesim.msg import Pose
from package_tutorial_rupp.msg import Aggregate

# This one unfortunately does not work (the callback is never executed)
class aggregator:
    def __init__(self):
        self.twist_sub = message_filters.Subscriber("/turtle1/cmd_vel", Twist)
        self.color_sub = message_filters.Subscriber("/turtle1/color_sensor", Color)
        self.pose_sub = message_filters.Subscriber("/turtle1/pose", Pose)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.twist_sub, self.color_sub, self.pose_sub], queue_size=10, slop = 0.1, allow_headerless=True)
        self.ts.registerCallback(self.callback)

    def callback(self, twist, color, pose):
        rospy.loginfo("In callback")
        message = Aggregate()
        message.twist = twist 
        message.color = color 
        message.pose = pose
        rospy.loginfo(message)
        rospy.loginfo("After callback")

# This one works!
class simple_agg:
    def __init__(self):
        self.message = Aggregate()
        rospy.Subscriber("turtle1/cmd_vel", Twist, self.twist_callback)
        rospy.Subscriber("turtle1/color_sensor", Color, self.color_callback)
        rospy.Subscriber("turtle1/pose", Pose, self.pose_callback)
    
    def twist_callback(self, twist):
        self.message.twist = twist 
    
    def color_callback(self, color):
        self.message.color = color 

    def pose_callback(self, pose):
        self.message.pose = pose 
        self.publish()
    
    def publish(self):
        pub = rospy.Publisher('rupp/state', Aggregate, queue_size = 1)
        rate_val = rospy.get_param(param_name="aggregator_rate", default=10)
        rate = rospy.Rate(rate_val) # 10hz default
        pub.publish(self.message)
        rate.sleep()

def main():
    name = rospy.get_param(param_name="aggregate_node_name", default="aggregator_custom")
    rospy.init_node(name, anonymous=True)
    agg = simple_agg()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__ == '__main__':
    main()