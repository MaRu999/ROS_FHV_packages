#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import Twist
from package_tutorial_rupp.srv import Stop_control

class config_publisher:
  def __init__(self):
    self.stop = 0
    self.pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 1)
    rate_val = rospy.get_param(param_name="publish_rate", default=10)
    self.rate = rospy.Rate(rate_val) # 10hz default
    self.stop_serv = rospy.Service('rupp/stop_control', Stop_control, self.handle_stop)

  def handle_stop(self, stop_msg):
    self.stop = stop_msg.stop
    return "Control started/stopped"

  def publish(self):
    twist = Twist() 
    twist.linear.x = rospy.get_param("bot_twist_linear_x", 2.0)
    twist.linear.y = rospy.get_param("bot_twist_linear_y", 0.0)
    twist.linear.z = rospy.get_param("bot_twist_linear_z", 0.0) 
    twist.angular.x = rospy.get_param("bot_twist_angular_x", 0.0)
    twist.angular.y = rospy.get_param("bot_twist_angular_y", 0.0)
    twist.angular.z = rospy.get_param("bot_twist_angular_z", 1.8)
    rospy.loginfo(twist)
    self.pub.publish(twist)
    self.rate.sleep()

def main():
    name = rospy.get_param("control_node_name", default="config_publish")
    rospy.init_node(name, anonymous=True)
    cfg_pub = config_publisher()
    try:
        while not rospy.is_shutdown():
          if cfg_pub.stop == 0:
            cfg_pub.publish()
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__ == '__main__':
  main()