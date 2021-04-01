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

# code from turtlebot3_examples -> nodes -> turtlebot3_obstacle
class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.wall_follow()

    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        return scan.ranges

    def wall_follow(self):
        twist = Twist()
        turtlebot_moving = True

        while not rospy.is_shutdown():
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)
            min_index = min(enumerate(lidar_distances), key=itemgetter(1))[0]
            ratio = 360/len(lidar_distances)
            ratioed_index = min_index*ratio
            angular = 0.0
            linear = LINEAR_VEL
            if min_distance < SAFE_STOP_DISTANCE * 2:
                if min(lidar_distances[:10]) < SAFE_STOP_DISTANCE or min(lidar_distances[354:364]) < SAFE_STOP_DISTANCE:
                    angular = math.radians(-90)
                    linear = 0.0
                else:
                    if ratioed_index > 180:
                        angular = math.radians(ratioed_index-360-90)
                    else:
                        angular = math.radians(ratioed_index-90)
            twist = Twist()
            twist.angular.z = angular
            twist.linear.x = linear
            self._cmd_pub.publish(twist)
            print(angular)

            

def main():
    rospy.init_node('rupp_wall_follow_complex')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()