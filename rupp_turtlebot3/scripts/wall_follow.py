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
        self.obstacle()

    def get_scan_try(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
        rf = scan.ranges[9]
        rdf = scan.ranges[44]
        r = scan.ranges[89]
        rdb = scan.ranges[134]
        rb = scan.ranges[169]
        lb = scan.ranges[189]
        ldb = scan.ranges[224]
        l = scan.ranges[269]
        ldf = scan.ranges[314]
        lf = scan.ranges[349]
        scan_filter.append(rf)
        scan_filter.append(rdf)
        scan_filter.append(r)
        scan_filter.append(rdb)
        scan_filter.append(rb)
        scan_filter.append(lb)
        scan_filter.append(ldb)
        scan_filter.append(l)
        scan_filter.append(ldf)
        scan_filter.append(lf)
        return scan_filter

    def obstacle(self):
        twist = Twist()
        turtlebot_moving = True

        while not rospy.is_shutdown():
            # order: rf, rdf, r, rdb, rb, lb, ldb, l, ldf, lf
            lidar_distances = self.get_scan_try()
            min_distance = min(lidar_distances)
            min_index = min(enumerate(lidar_distances), key=itemgetter(1))[0]
            lf = lidar_distances[9]
            left = lidar_distances[2]
            if lf < SAFE_STOP_DISTANCE:
                print("Turning right...")
                twist.linear.x = 0.0
                twist.angular.z = -math.pi/2
                self._cmd_pub.publish(twist)
            # case: wall to the left -> drive along wall (should work on both straight and curved obstacles)
            #elif left > 3.0 and left < 3.4:
             #   print("Driving along wall...")
              #  twist.angular.z = 1/(1 - (0.05 - left))
               # twist.linear.x = (1 / (1 - (0.05 - left)))/2
                #self._cmd_pub.publish(twist)
                # right_val = (1/(2-(0.05 - left)))/2 + 1/(1 - (0.05 - ldf)) + 1/(1 - (0.05 - lb))
            # base case: follow line (simply drives straight slowly while not on the line)
            else:
                print("Going straight...")
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
                if left > 3.0 and left < 3.4:
                    twist.angular.x = LINEAR_VEL
                    twist.angular.z = math.pi/8
                self._cmd_pub.publish(twist)

def main():
    rospy.init_node('turtlebot3_obstacle_test')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()