#!/usr/bin/env python

import rospy 
import math 
from operator import itemgetter
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
MIN_DISTANCE = 3.5

class GoTo:
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan()

    def scan(self):
        while not rospy.is_shutdown():
            scan = rospy.wait_for_message('scan', LaserScan)
            min_distance = min(scan.ranges)
            min_index = min(enumerate(scan.ranges), key=itemgetter(1))[0]
            ratio = 360/len(scan.ranges)
            ratioed_index = min_index * ratio
            linear = LINEAR_VEL
            if min_distance < MIN_DISTANCE and min_distance > SAFE_STOP_DISTANCE:
                if ratioed_index > 180:
                    angular = math.radians(ratioed_index-360)
                else:
                    angular = math.radians(ratioed_index)
            elif min_distance < SAFE_STOP_DISTANCE:
                linear = 0.0
                angular = 0.0
            else:
                angular = 0.0
            twist = Twist()
            twist.angular.z = angular 
            twist.linear.x = linear
            self._cmd_pub.publish(twist)
            print(angular)

def main():
    rospy.init_node('goto_nearest')
    try:
        gt = GoTo()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()